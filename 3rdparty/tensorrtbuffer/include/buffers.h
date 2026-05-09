/*
 * SPDX-FileCopyrightText: Copyright (c) 1993-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * --- AirSLAM Jazzy port note ---
 * Migrated to the TensorRT 10 explicit-tensor API:
 *   - getNbBindings()              -> getNbIOTensors()
 *   - getBindingIndex(name)        -> name-keyed map lookup
 *   - getBindingDimensions(i)      -> getTensorShape(name)
 *   - getBindingDataType(i)        -> getTensorDataType(name)
 *   - getBindingVectorizedDim(i)   -> getTensorVectorizedDim(name)
 *   - bindingIsInput(i)            -> getTensorIOMode(name) == kINPUT
 *   - executeV2(bindings)          -> setTensorAddress(name,ptr) + enqueueV3
 *   - hasImplicitBatchDimension()  -> removed (implicit batch gone in TRT 10)
 */
#ifndef TENSORRT_BUFFERS_H
#define TENSORRT_BUFFERS_H

#include "NvInfer.h"
#include "common.h"
#include "half.h"
#include "safe_common.h"
#include <cassert>
#include <cuda_runtime_api.h>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <new>
#include <numeric>
#include <string>
#include <vector>

namespace tensorrt_buffer
{

//!
//! \brief  The GenericBuffer class is a templated class for buffers.
//!
    template <typename AllocFunc, typename FreeFunc>
    class GenericBuffer
    {
    public:
        GenericBuffer(nvinfer1::DataType type = nvinfer1::DataType::kFLOAT)
                : mSize(0)
                , mCapacity(0)
                , mType(type)
                , mBuffer(nullptr)
        {
        }

        GenericBuffer(size_t size, nvinfer1::DataType type)
                : mSize(size)
                , mCapacity(size)
                , mType(type)
        {
            if (!allocFn(&mBuffer, this->nbBytes()))
            {
                throw std::bad_alloc();
            }
        }

        GenericBuffer(GenericBuffer&& buf)
                : mSize(buf.mSize)
                , mCapacity(buf.mCapacity)
                , mType(buf.mType)
                , mBuffer(buf.mBuffer)
        {
            buf.mSize = 0;
            buf.mCapacity = 0;
            buf.mType = nvinfer1::DataType::kFLOAT;
            buf.mBuffer = nullptr;
        }

        GenericBuffer& operator=(GenericBuffer&& buf)
        {
            if (this != &buf)
            {
                freeFn(mBuffer);
                mSize = buf.mSize;
                mCapacity = buf.mCapacity;
                mType = buf.mType;
                mBuffer = buf.mBuffer;
                buf.mSize = 0;
                buf.mCapacity = 0;
                buf.mBuffer = nullptr;
            }
            return *this;
        }

        void* data() { return mBuffer; }
        const void* data() const { return mBuffer; }
        size_t size() const { return mSize; }

        size_t nbBytes() const
        {
            return this->size() * tensorrt_buffer::getElementSize(mType);
        }

        void resize(size_t newSize)
        {
            mSize = newSize;
            if (mCapacity < newSize)
            {
                freeFn(mBuffer);
                if (!allocFn(&mBuffer, this->nbBytes()))
                {
                    throw std::bad_alloc{};
                }
                mCapacity = newSize;
            }
        }

        void resize(const nvinfer1::Dims& dims)
        {
            return this->resize(tensorrt_buffer::volume(dims));
        }

        ~GenericBuffer()
        {
            freeFn(mBuffer);
        }

    private:
        size_t mSize{0}, mCapacity{0};
        nvinfer1::DataType mType;
        void* mBuffer;
        AllocFunc allocFn;
        FreeFunc freeFn;
    };

    class DeviceAllocator
    {
    public:
        bool operator()(void** ptr, size_t size) const
        {
            return cudaMalloc(ptr, size) == cudaSuccess;
        }
    };

    class DeviceFree
    {
    public:
        void operator()(void* ptr) const { cudaFree(ptr); }
    };

    class HostAllocator
    {
    public:
        bool operator()(void** ptr, size_t size) const
        {
            *ptr = malloc(size);
            return *ptr != nullptr;
        }
    };

    class HostFree
    {
    public:
        void operator()(void* ptr) const { free(ptr); }
    };

    using DeviceBuffer = GenericBuffer<DeviceAllocator, DeviceFree>;
    using HostBuffer = GenericBuffer<HostAllocator, HostFree>;

//!
//! \brief  The ManagedBuffer class groups together a pair of corresponding device and host buffers.
//!
    class ManagedBuffer
    {
    public:
        DeviceBuffer deviceBuffer;
        HostBuffer hostBuffer;
    };

//!
//! \brief  BufferManager — TRT 10 tensor-name-keyed buffer pool.
//!
//! \details Owns one pair of (device, host) buffers per I/O tensor of an
//!          ICudaEngine. Sized at construction from the engine's static shapes
//!          (or, if a context is provided, the context's currently set shapes
//!          for inputs with dynamic dimensions).
//!
//!          Inference flow with this class is:
//!              1. Resize input host buffers if shapes are dynamic, copy data
//!                 into them, then call copyInputToDeviceAsync(stream).
//!              2. Call setTensorAddresses(context) once to wire device buffers
//!                 onto the execution context.
//!              3. Call context->enqueueV3(stream).
//!              4. Call copyOutputToHostAsync(stream) and synchronize.
//!
    class BufferManager
    {
    public:
        static const size_t kINVALID_SIZE_VALUE = ~size_t(0);

        //!
        //! \brief Create a BufferManager for the given engine.
        //! \param engine   shared engine pointer (ownership shared).
        //! \param context  optional execution context — if non-null, dynamic
        //!                 input shapes are read from it (must already be set
        //!                 via setInputShape()) so dynamic tensors get a
        //!                 correctly-sized buffer.
        //!
        BufferManager(std::shared_ptr<nvinfer1::ICudaEngine> engine,
                      const nvinfer1::IExecutionContext* context = nullptr)
                : mEngine(engine)
        {
            const int32_t nbTensors = mEngine->getNbIOTensors();
            mTensorNames.reserve(nbTensors);
            for (int32_t i = 0; i < nbTensors; ++i)
            {
                const char* nameRaw = mEngine->getIOTensorName(i);
                std::string name(nameRaw);
                auto dims = context ? context->getTensorShape(nameRaw)
                                    : mEngine->getTensorShape(nameRaw);
                nvinfer1::DataType type = mEngine->getTensorDataType(nameRaw);
                int32_t vecDim = mEngine->getTensorVectorizedDim(nameRaw);
                size_t vol = 1;
                if (-1 != vecDim)
                {
                    int32_t scalarsPerVec = mEngine->getTensorComponentsPerElement(nameRaw);
                    dims.d[vecDim] = divUp(dims.d[vecDim], scalarsPerVec);
                    vol *= scalarsPerVec;
                }
                vol *= tensorrt_buffer::volume(dims);
                auto manBuf = std::make_unique<ManagedBuffer>();
                manBuf->deviceBuffer = DeviceBuffer(vol, type);
                manBuf->hostBuffer = HostBuffer(vol, type);
                mManagedBuffers.emplace(name, std::move(manBuf));
                mTensorNames.push_back(name);
            }
        }

        //!
        //! \brief Wire every I/O tensor's device buffer onto the execution
        //!        context. Must be called before context->enqueueV3(stream).
        //!
        bool setTensorAddresses(nvinfer1::IExecutionContext* context) const
        {
            for (const auto& name : mTensorNames)
            {
                void* devPtr = mManagedBuffers.at(name)->deviceBuffer.data();
                if (!context->setTensorAddress(name.c_str(), devPtr))
                {
                    return false;
                }
            }
            return true;
        }

        //!
        //! \brief Returns the device buffer corresponding to tensorName.
        //!        Returns nullptr if no such tensor can be found.
        //!
        void* getDeviceBuffer(const std::string& tensorName) const
        {
            return getBuffer(false, tensorName);
        }

        //!
        //! \brief Returns the host buffer corresponding to tensorName.
        //!        Returns nullptr if no such tensor can be found.
        //!
        void* getHostBuffer(const std::string& tensorName) const
        {
            return getBuffer(true, tensorName);
        }

        //!
        //! \brief Returns the size of the host and device buffers that
        //!        correspond to tensorName, or kINVALID_SIZE_VALUE.
        //!
        size_t size(const std::string& tensorName) const
        {
            auto it = mManagedBuffers.find(tensorName);
            if (it == mManagedBuffers.end()) return kINVALID_SIZE_VALUE;
            return it->second->hostBuffer.nbBytes();
        }

        //!
        //! \brief Resize the device + host pair backing tensorName. Use after
        //!        setInputShape() on the context if input dims changed.
        //!
        void resize(const std::string& tensorName, const nvinfer1::Dims& dims)
        {
            auto it = mManagedBuffers.find(tensorName);
            if (it == mManagedBuffers.end()) return;
            it->second->deviceBuffer.resize(dims);
            it->second->hostBuffer.resize(dims);
        }

        //!
        //! \brief Templated print function that dumps buffers of arbitrary type to std::ostream.
        //!
        template <typename T>
        void print(std::ostream& os, void* buf, size_t bufSize, size_t rowCount)
        {
            assert(rowCount != 0);
            assert(bufSize % sizeof(T) == 0);
            T* typedBuf = static_cast<T*>(buf);
            size_t numItems = bufSize / sizeof(T);
            for (int i = 0; i < static_cast<int>(numItems); i++)
            {
                if (rowCount == 1 && i != static_cast<int>(numItems) - 1)
                    os << typedBuf[i] << std::endl;
                else if (rowCount == 1)
                    os << typedBuf[i];
                else if (i % rowCount == 0)
                    os << typedBuf[i];
                else if (i % rowCount == rowCount - 1)
                    os << " " << typedBuf[i] << std::endl;
                else
                    os << " " << typedBuf[i];
            }
        }

        void copyInputToDevice() { memcpyBuffers(true, false, false); }
        void copyOutputToHost()  { memcpyBuffers(false, true, false); }

        void copyInputToDeviceAsync(const cudaStream_t& stream = 0)
        {
            memcpyBuffers(true, false, true, stream);
        }

        void copyOutputToHostAsync(const cudaStream_t& stream = 0)
        {
            memcpyBuffers(false, true, true, stream);
        }

        ~BufferManager() = default;

    private:
        void* getBuffer(const bool isHost, const std::string& tensorName) const
        {
            auto it = mManagedBuffers.find(tensorName);
            if (it == mManagedBuffers.end()) return nullptr;
            return isHost ? it->second->hostBuffer.data()
                          : it->second->deviceBuffer.data();
        }

        void memcpyBuffers(const bool copyInput, const bool deviceToHost,
                           const bool async, const cudaStream_t& stream = 0)
        {
            for (const auto& name : mTensorNames)
            {
                const auto ioMode = mEngine->getTensorIOMode(name.c_str());
                const bool isInput = (ioMode == nvinfer1::TensorIOMode::kINPUT);
                if (copyInput != isInput) continue;

                auto& mb = *mManagedBuffers.at(name);
                void* dstPtr = deviceToHost ? mb.hostBuffer.data()
                                            : mb.deviceBuffer.data();
                const void* srcPtr = deviceToHost ? mb.deviceBuffer.data()
                                                  : mb.hostBuffer.data();
                const size_t byteSize = mb.hostBuffer.nbBytes();
                const cudaMemcpyKind memcpyType = deviceToHost
                        ? cudaMemcpyDeviceToHost : cudaMemcpyHostToDevice;
                if (async)
                    CHECK(cudaMemcpyAsync(dstPtr, srcPtr, byteSize, memcpyType, stream));
                else
                    CHECK(cudaMemcpy(dstPtr, srcPtr, byteSize, memcpyType));
            }
        }

        std::shared_ptr<nvinfer1::ICudaEngine> mEngine;
        std::map<std::string, std::unique_ptr<ManagedBuffer>> mManagedBuffers;
        std::vector<std::string> mTensorNames; // preserves engine I/O order
    };

} // namespace tensorrt_buffer

#endif // TENSORRT_BUFFERS_H
