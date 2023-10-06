== 2023-03-05 - From Zero to Triangle

Since I'll be rolling my own engine to get the most out of older computers, I need to build the systems a game engine would normally provide. I decided to start development with the renderer, as I felt it was going to be the most complicated of the engine systems, and being able to show things on screen will make future development easier to debug. Before starting there were a couple design decisions I needed to make, such as whether to do the rendering on the CPU or GPU.

#table(
    columns: (auto, auto, auto),
    inset: 10pt,
    align: horizon,
    [*Option*], [*Advantages*], [*Disadvantages*],
    [CPU rendering],
    [
        - No need for a complex graphics API
        - No cross-processor data transfer
    ],
    [
        - Doesn't take advantage of a coprocessor designed for graphics
        - Limits CPU time available for non-graphics tasks
    ],
    [GPU rendering],
    [
        - Often faster than CPU rendering
        - Means the CPU has more time to do other tasks
        - Data is always accessible
    ],
    [
        - Complex APIs
        - Cross-processor data transfer is slow
    ]
)

I decided on GPU rendering despite the extra complexity, as it moves a lot of the work to a coprocessor which will lower the CPU requirements as it will have more time to process other systems such as UI and networking. GPU rendering would also allow me to be more ambitious with the game's graphics as it tends to be more powerful for this than a CPU for this purpose. The downside to this decision is that I'd have to learn a complex graphics API as well as a shader language, which was my next decision.

#table(
    columns: (auto, auto, auto),
    inset: 10pt,
    align: horizon,
    [*Option*], [*Advantages*], [*Disadvantages*],
    [DirectX],
    [
        - Most widely used graphics API
        - Tends to be the best optimised due to its wide use
    ],
    [
        - Only works on Windows
        - Also requires me to learn a lot about the Windows API
        - Fairly complex API
    ],
    [Metal],
    [
        - Only graphics API that MacOS natively supports
    ],
    [
        - Only work on MacOS
        - Poorly documented
        - Forces you to use a prioprietary shader language
    ],
    [OpenGL],
    [
        - Simple API
        - Runs on Windows, MacOS, Linux and some mobile platforms
    ],
    [
        - Old API that tends to be outperformed by newer ones
        - Uses a global context/ state machine model that makes it very hard to debug
        - A lot of behaviour in other graphics APIs are considered optional extensions in OpenGL, such as blending
    ],
    [Vulkan],
    [
        - Runs natively on Windows, Linux and some mobile platforms
        - Second-best optimised graphics API after DirectX
        - Runs non-natively on MacOS using MoltenVK
        - Wide array of extensions for things like hardware raytracing
    ],
    [
        - Very complex API
    ]
)

With this research done and my goals of running on as many devices as possible in mind, I decided to use Vulkan as it can be run on any desktop platform and its well optimised, and also gives me the oppotunity to explore recent graphics developments such as raytracing. With that decided I found some online tutorials and guides on Vulkan and started writing the graphics code.

Vulkan is written in C, and my program is written in Rust, which are compatible using the Rust FFI(Foreign Function Interface), however directly interfacing with C apis in Rust results in a lot of boilerplate so I'd want to use an existing boilerplate library. I chose one called `ash` which provided a minimal layer over the Vulkan API, I chose it over a higher-level library such as `vulkano` because I wanted the extra control over my graphics code.

=== Instances

One of the best parts of Vulkan is that there is no global state, all shared data is handled with the `VkInstance` object. Creating a `VkInstance` object initialises the Vulkan API and allows the application to pass information about itself to the implementation. It also allows the application to pass in extensions and validations layers that it plans to use. 

Extensions are vulkan's way of enabling features that not all GPU manufacturers need to implement, such as raytracing, which wouldn't be needed in compute-focused GPUs such as the ones being used to train AIs. The presentation system, which is responsible for allowing a program to present images to the screen, is also behind an extension as not all GPUs need to be able present to the screen, because of this I will need to enable some extensions.

Vulkan is a very complex API and correct usage is difficult to achieve without reading the 2000+ pages of documentation surrounding it. Many APIs resolve this by adding computationally expensive validation checks to ensure correct usage, but this has a lot of overhead and makes assumptions on what to do when something goes wrong. Khronos resolves these issues using what the call validation layers, which are optional layers are inserted at instance creation which intercept any API calls and log any issues with usage. Many other comapnies have made validation layers such as LunarG who have created a API dump layer which logs any API calls to the console for debugging.

