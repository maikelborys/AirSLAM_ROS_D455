// Compile-only smoke test: ensures the TRT 10 migrated buffers.h header
// instantiates cleanly. Built by the tensorrtbuffer CMakeLists.
#include "buffers.h"

#include <memory>

int main()
{
    // We don't run anything — just force template instantiation of the
    // buffer types so the header is exercised at compile time.
    using DBuf = tensorrt_buffer::DeviceBuffer;
    using HBuf = tensorrt_buffer::HostBuffer;
    DBuf d;
    HBuf h;
    (void)d;
    (void)h;
    // BufferManager construction takes a real ICudaEngine; we can only
    // reference the type here, not instantiate it without TRT runtime.
    using BM = tensorrt_buffer::BufferManager;
    (void)sizeof(BM);
    return 0;
}
