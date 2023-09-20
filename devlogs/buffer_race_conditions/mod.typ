== 2023-07-25 - Buffer Race Conditions
=== The Problem
Occasionally, especially on my desktop, I'd notice missing chunks of vertex data. Upon investigation with RenderDoc I saw that the first 64 bytes of the vertex buffer was being set to a strange constant value. 

=== Debugging
Using RenderDoc I traced back into the memory arena that buffer allocations were coming from, the same memory offset was being assigned to two buffers in the same frame.
#image("renderdoc.png")

Now this in theory wouldn't be an issue since the first buffer was being used in the render pass, and the second in the UI pass, however the GPU wasn't waiting properly when I was allocating and writing memory, even when I told the device to wait till idle on every write (which incurred a heavy performance penalty). The issue was due to the fact I was deallocating memory according to Rust lifetimes, i.e. when a resource goes out of scope rather than GPU lifetimes which were a lot longer, hence the race condition.

=== The Fix
The solution was to record frees into a "to free" buffer and flush that buffer when the program could guarantee the GPU wouldn't be using any resources and with this the vertex buffer corruption seemed to disappear. It's not the best solution and it locks me out of frames-in-flight so I may need to revisit it, however that won't be too hard since part of my debugging resulted in me writing a custom allocator and dropping and gpu_allocator requirement, so it will be easier to extend in future.

Here's an example of batching frees:
#image("batched.png")

TODO: Add code snippets and more detail