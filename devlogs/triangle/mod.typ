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

Vulkan is a very complex API and correct usage is difficult to achieve without reading the 2000+ pages of documentation surrounding it. Many APIs resolve this by adding computationally expensive validation checks to ensure correct usage, but this has a lot of overhead and makes assumptions on what to do when something goes wrong. Khronos resolves these issues using what they call validation layers, which are optional layers are inserted at instance creation which intercept any API calls and log any issues with usage. Many other comapnies have made validation layers such as LunarG who have created a API dump layer which logs any API calls and their parameters to the console for debugging. I'm planning to use validation layers in my program to ensure valid usage of the vulkan API, however I'll only be enabling them when I'm debugging, so the performance won't be impacted for release builds of the game.

The final piece of information required to create the instance is the application info, which includes the name and version of both the game and engine as well as the minimum version of vulkan the application will work with.

With extensions, validations layers and application info understood, I had everything needed to write the code for instance creation, I started with application info:

```pretty-rs
let app_info = vk::ApplicationInfo::builder()
    .application_name(cstr!("aetheria"))
    .application_version(vk::make_api_version(0, 1, 0, 0))
    .engine_name(cstr!("aetheria"))
    .engine_version(vk::make_api_version(0, 1, 0, 0))
    .api_version(vk::make_api_version(0, 1, 3, 238));
```

The `ash` crate generates builder patterns for every structure in the Vulkan API, which lets me replace struct initialisation with a series of method calls, the `ash` create also uses to add some type conversions between what I'm calling and what actually gets passed to the Vulkan API, for instance here it allows me to pass a nicer `&CStr` for the application name rather than the actual field type which is a `*const c_char`, a raw pointer. The `cstr!("CONTENT")` syntax is a nice macro from the `cstr` create that lets me reduce some boilerplate `CStr` initialisation code, it just creates a `&'static CStr` with the contents of whatever string is passed into it. The calls to `vk::make_api_version(...)` on lines 3, 5 and 6 are used because Vulkan uses a strange version encoding to fit a variant, major, minor and patch version (which are the arguments) into one 32 bit unsigned integer, the `make_api_version` function just encodes the versions correctly. I chose Vulkan 1.3.238 which at the time of writing was the version installed on my computer, I chose the latest version as I wanted all of the features available to me, once I finish the graphics code I will likely come back here and reduce the version number so more devices are supported.

Next I had to handle enabling extensions and validation layers, to do this I first had to fetch a list of extensions and layers supported by the implementation installed on the computer, this I had to check that all the extensions and layers I wanted were available, and if they were to enable them. 

```pretty-rs
let available_layers = entry.enumerate_instance_layer_properties()?;
let available_extensions = entry.enumerate_instance_extension_properties(None)?;

let available_layer_names: Vec<&CStr> = available_layers
    .iter()
    .map(|layer| CStr::from_bytes_until_nul(cast_slice(&layer.layer_name)).unwrap())
    .collect();

let available_extension_names: Vec<&CStr> = available_extensions
    .iter()
    .map(|extension| {
        CStr::from_bytes_until_nul(cast_slice(&extension.extension_name)).unwrap()
    })
    .collect();

let wanted_layers = super::get_wanted_layers();
let wanted_extensions = get_wanted_extensions();

let wanted_layers = super::intersection(&wanted_layers, &available_layer_names);
let wanted_extensions = super::intersection(&wanted_extensions, &available_extension_names);

info!("Using instance layers: {:?}", wanted_layers);
info!("Using instance extensions: {:?}", wanted_extensions);

let wanted_layers_raw: Vec<*const i8> =
    wanted_layers.iter().map(|name| name.as_ptr()).collect();
let wanted_extensions_raw: Vec<*const i8> =
    wanted_extensions.iter().map(|name| name.as_ptr()).collect();
```

Lines 1 and 2 call the Vulkan API to get a list of available layers and extensions, lines 4-15 take the extension and layer information get the names of each, turning them into `&CStr` objects. Then `get_wanted_extensions` and `get_wanted_layers` are called to get a static list of names of wanted extensions/layers, the intersection of the set of available extensions and the set of wanted extensions are found, and then turned back into a list of `Vec<*const i8>` ready to be passed back into the Vulkan API for instance creation. Currently my wanted layers is just Khronos' validation layer, and my wanted extensions list is empty, since there are no extensions I want to use yet.  

The instance creation code is quite simply in comparison and just takes information already created before passing it off the Vulkan API:

```pretty-rs
let instance_info = vk::InstanceCreateInfo::builder()
    .application_info(&app_info)
    .enabled_layer_names(&wanted_layers_raw)
    .enabled_extension_names(&wanted_extensions_raw);

let instance = unsafe { entry.create_instance(&instance_info, None)? };
```

The unsafe block is needed since the Vulkan API is all written in C, which is an unsafe language. Now if everything works correctly, `instance` now stores a handle to a `VkInstance` object and thevVulkan API can be used.

I tested this code by adding LunarG's api dump validation layer, which will log the call to `create_instance`, running the application yielded:

```
vkCreateInstance(pCreateInfo, pAllocator, pInstance) returns VkResult VK_SUCCESS (0):
    pCreateInfo:                    const VkInstanceCreateInfo* = 0x7fff89b19290:
        sType:                          VkStructureType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO (1)
        pNext:                          const void* = NULL
        flags:                          VkInstanceCreateFlags = 0
        pApplicationInfo:               const VkApplicationInfo* = 0x7fff89b1cf00:o
            sType:                          VkStructureType = VK_STRUCTURE_TYPE_APPLICATION_INFO (0)
            pNext:                          const void* = NULL
            pApplicationName:               const char* = "aetheria"
            applicationVersion:             uint32_t = 4194304
            pEngineName:                    const char* = "aetheria"
            engineVersion:                  uint32_t = 4194304
            apiVersion:                     uint32_t = 4206830
        enabledLayerCount:              uint32_t = 2
        ppEnabledLayerNames:            const char* const* = 0x5571fe7341c0
            ppEnabledLayerNames[0]:         const char* const = "VK_LAYER_LUNARG_api_dump"
            ppEnabledLayerNames[1]:         const char* const = "VK_LAYER_KHRONOS_validation"
        enabledExtensionCount:          uint32_t = 0
        ppEnabledExtensionNames:        const char* const* = 0x5571fe7341e0
        pNext:                          const void* = NULL
    pAllocator:                     const VkAllocationCallbacks* = NULL
    pInstance:                      VkInstance* = 0x5571fe7312e0
```

From this I can tell this code is working correctly, because the instance create info contains both validation layer names as well as the correct application info, the versions numbers are a bit odd because of the encoding Vulkan uses for versions. With instance creation complete I moved on to physical devices

=== Physical Devices

Physical devices are how Vulkan represents a piece of GPU hardware, having a `VkPhysicalDevice` handle allows the application to request properties and capabilities of the GPU in order to check if the GPU is suitable for the application's needs. I decided the encapsulate the `VkPhysicalDevice` handle in a struct I called `PhysicalDevice`, which fetched and stored all of the data for a device when it's created to avoid constantly fetching the same data.

```pretty-rs
#[derive(Debug, Clone)]
pub struct PhysicalDevice {
    pub(crate) physical: vk::PhysicalDevice,
    pub properties: PhysicalDeviceProperties,
    pub queue_families: Vec<vk::QueueFamilyProperties>,
    pub features: vk::PhysicalDeviceFeatures,
}

impl PhysicalDevice {
    unsafe fn new(instance: &Instance, physical: vk::PhysicalDevice) -> Self {
        let properties = instance.get_physical_device_properties(physical);
        let properties = PhysicalDeviceProperties::new(&properties);
        let queue_families = instance.get_physical_device_queue_family_properties(physical);
        let features = instance.get_physical_device_features(physical);

        Self {
            physical,
            properties,
            queue_families,
            features,
        }
    }
}
```

The struct stores a handle to the physical device, a `PhysicalDeviceProperties` instance (an encapsulation I built to turn any string properties into nice Rust `Strings`) which contains static properties such as the device's name, or whether its a discrete or integrated GPU, etc. The `vk::PhysicalDeviceFeatures` instance stores a list of possible features and whether they are supported on the device, such as anisotropic filtering. And finally it stores a vector of `QueueFamilyProperties` instances, which I'll come to when I talk about queues later.

With physical devices encapsulated, I now needed a way of fetching device handles and instantiating the struct, I created a method called `get_physical_devices` on `Instance`

```pretty-rs
impl Instance {
  ...
  
  pub fn get_physical_devices(&self) -> Result<Vec<PhysicalDevice>, vk::Result> {
    let physicals = unsafe { self.enumerate_physical_devices()? };
    unsafe {
      Ok(physicals
        .iter()
        .cloned()
        .map(|physical| PhysicalDevice::new(self, physical))
        .collect())
    }
  }
}
```

I needed to test all this code, so I wrote some temporary entrypoint code that loads Vulkan, creates an `Instance` and then gets a `PhysicalDevice`:

```pretty-rs
use ash::Entry;

mod vulkan;

fn main() {
  let entry = Entry::linked();
  let instance = vulkan::Instance::new(&entry).expect("Vulkan instance creation failed");
  let physicals = instance.get_physical_devices().expect("Fetching physical devices failed");
  let physical = physicals.first().expect("No vulkan compatible devices");
  println!("{}", physical.properties.device_name);
}
```

The output of the program will depend on the GPUs in the computer running it, but on my desktop it prints `NVIDIA GeForce GTX 1050 Ti`, which is my discrete GPU for my desktop. Because `PhysicalDevice` derives the `Debug` trait, I am also able to print out the `PhysicalDevice` object itself, which I did during testing, however the output is very long so I won't show it here.

=== Surfaces

Surfaces are the first step in Vulkan's presentation system, they are a platform-independant handle to a window, which can then be used to present image to, as well as getting information about the window such as width and height. Like the rest of the presentation system, surfaces are behind an extension, in this case the `VK_KHR_surface` extension, in addition there are also operating system specific extensions for surface creation, for Linux I chose `VK_KHR_xlib_surface`, I'm ignoring MacOS at the moment as I don't have a mac to test on, and I'll add windows support at a later point. I started by adding these extensions to the `get_wanted_extension` function:

```pretty-rs
#[cfg(target_os = "linux")]
fn get_wanted_extensions() -> Vec<&'static CStr> {
    vec![khr::Surface::name(), khr::XlibSurface::name()]
}
```

The `ash` create provides some convenient functions for getting an extension's name to avoid typos, which is what I use here. Line 1 makes it so that this function is only declared when compiling for Linux, so I can have platform specific code that is swapped out at compile time. With that done the extensions are being added when the instance is created, but they still have to be loaded, as the address of the functions for an extension are unknown. The `ash` create handles loading extensions by creating a struct that encapsulates all of an extension's functions, its these structs that have the `name` methods I used above. Because these instances have to be stores somewhere I created an `InstanceExtensions` struct:

```pretty-rs
#[derive(Clone)]
pub struct InstanceExtensions {
    pub surface: Option<khr::Surface>,
    pub xlib_surface: Option<khr::XlibSurface>,
}

impl InstanceExtensions {
  pub fn load(entry: &ash::Entry, instance: &ash::Instance, available: &[&CStr]) -> Self {
    Self {
      surface: available
        .iter()
        .find(|ext| **ext == khr::Surface::name())
        .map(|_| khr::Surface::new(entry, instance)),
      xlib_surface: available
        .iter()
        .find(|ext| **ext == khr::XlibSurface::name())
        .map(|_| khr::XlibSurface::new(entry, instance)),
    }
  }
}
```

The struct stores the extensions in an `Option<T>` enum so I can have optional extensions in the future, the `InstanceExtension::load` method also checks if the extensions are available before loading them, so the program won't crash. I added loading extensions to instance creation:

```pretty-rs
pub struct Instance {
  instance: ash::Instance,
  pub extensions: InstanceExtensions
}

impl Instance {
  pub fn new(entry: &ash::Entry) {
    ...

    Ok(Self {
      extensions: InstanceExtensions::load(entry, &instance, &available_extension_names),
      instance
    })
  }
}
```

The last thing I needed before creating a surface was to actually create the window I was going to use to present the game to the user, I decided to use a crate called `winit` for this. `winit` has two main objects, the `Window` and the `EventLoop` which are created as shown below:

```pretty-rs
pub fn main() {
  let event_loop = winit::event_loop::EventLoop::new();
  let window = winit::window::WindowBuilder::new()
    .build(&event_loop)
    .unwrap();
  
  ...
}
```

Once everything else is initialised, the application begins its main loop using winit's `EventLoop::run` method:

```pretty-rs 
pub fn main() {
  ...

  event_loop.run(move |event, _, control_flow| {
    control_flow.set_poll();
    match event {
      winit::event::Event::WindowEvent {
        event: winit::event::WindowEvent::CloseRequested,
        ..
      } => {
        control_flow.set_exit();
      }
      winit::event::Event::DeviceEvent { event: winit::event::DeviceEvent::Key(input), .. } => {
        if let Some(key) = input.virtual_keycode && key == winit::event::VirtualKeyCode::Escape {
          control_flow.set_exit();
        }
      }
      _ => {}
    }
  })
}
```

This event handler closes the window if the user presses the operating system's close button, or if the Escape key is pressed. It also sets the control flow to Poll, meaning the event loop with be run each frame, which is needed for a video game. With the window initialised I now moved on to surface creation.

To create the surface for Linux, I needed to fill out a `XlibSurfaceCreateInfo` struct which took some raw handles to the x11 window, then I passed it into the xlib surface extension and got the surface handle from that:

```pretty-rs
pub struct Surface {
  pub(crate) surface: vk::SurfaceKHR,
}

impl Surface {
  #[cfg(target_os = "linux")]
  pub fn new(instance: &Instance, window: &Window) -> Result<Self, vk::Result> {
    let create_info = vk::XlibSurfaceCreateInfoKHR::builder()
      .dpy(window.xlib_display().unwrap() as *mut *const c_void)
      .window(window.xlib_window().unwrap());

    let surface = unsafe {
      instance
        .extensions
        .xlib_surface
        .as_ref()
        .unwrap()
        .create_xlib_surface(&create_info, None)?
    };

    Ok(Self { surface })
  }
}
```

With the surface constructed, the next step was to create the logical device and some queues.

=== Execution Model

Before I get to logical devices, I needed to do some research into Vulkan's execution model so I understood what queues were. All operations in Vulkan have to be recorded into `CommandBuffer` objects, from drawing to rescaling an image. Once these `CommandBuffer` objects are recorded they can be submitted to a `Queue` on the GPU, queues are a bit like threads but on a GPU, each queue can have a `CommandBuffer` submitted, so multiple `CommandBuffer` object can be executed at once. For this program I am only using one queue for graphics so rendering will be executed like a single-threaded program, I decided this because of the extra complexity multi-threaded GPU code would create. Each queue belongs to a queue family, which is a collection of queues with the same capabilities, such as presenting, or running compute shaders. Using the CPU analogy, queues would be the threads and queue families would be the cores. 

Now I knew what `Queue`s were, I was ready to create a logical device.

=== Devices and Queues

Devices are the most important structure in Vulkan, they represent a handle to a device the program can use to execute command buffers, draw pretty picture and present to the screen. Looking at the `ash` documentation for `DeviceCreateInfo`:

```pretty-rs
pub struct DeviceCreateInfo {
    pub s_type: StructureType,
    pub p_next: *const c_void,
    pub flags: DeviceCreateFlags,
    pub queue_create_info_count: u32,
    pub p_queue_create_infos: *const DeviceQueueCreateInfo,
    pub enabled_layer_count: u32,
    pub pp_enabled_layer_names: *const *const c_char,
    pub enabled_extension_count: u32,
    pub pp_enabled_extension_names: *const *const c_char,
    pub p_enabled_features: *const PhysicalDeviceFeatures,
}
```

Ignoring `s_type`, `p_next` and `flags`, I needed 4 things for the `DeviceCreateInfo`: an array of `DeviceQueueCreateInfo`, an array of enabled validation layers, an array of enabled device extensions, and finally a refernce to a `PhysicalDeviceFeatures` object.

I'm skipping over extensions and validation layers because the code is very similar to `Instance` creation above.

For a basic Vulkan program, I need 2 queues, one with the capabilities to render graphics, and another with the capabilities to present. These can be the same queues, but they aren't always so I need to write the code that works for either situation. To find out whether a queue family supports a certain capability I have to iterate over all families and check if they support the features I need, the code to do this is below:

```pretty-rs
let (graphics_family_index, graphics_family) = physical
    .queue_families
    .iter()
    .enumerate()
    .find(|(_, family)| family.queue_flags.intersects(vk::QueueFlags::GRAPHICS))
    .expect("No graphics queue family");

let (present_family_index, present_family) = physical
    .queue_families
    .iter()
    .enumerate()
    .find(|(i, _)| {
	instance
	    .extensions
	    .surface
	    .as_ref()
	    .unwrap()
	    .get_physical_device_surface_support(
		physical.physical,
		(*i).try_into().unwrap(),
		surface.surface,
	    )
	    .unwrap()
    })
    .expect("No present family");
```

The polling for a presentation queue is much more complex as it requires using a method from the `VK_KHR_surface` extension. I've also kept both the index of the queue family as well as the properties of the queue family, as both will be needed later. The program won't function without a present and graphics queue so I crash out at this point if a suitable queue family can't be found.

Eventually I will have to revisit this section to add other types of queues such as compute queues if I want to use compute pipleines.

Now I've selected the queue families I want to get queues from, I have to fill out the `VkDeviceQueueCreateInfo` structs which are passed into the `VkDeviceCreateInfo` struct to initialise queues. The `VkDeviceQueueCreateInfo` struct comprises of:

```pretty-rs
pub struct DeviceQueueCreateInfo {
    pub queue_family_index: u32,
    pub queue_count: u32,
    pub p_queue_priorities: *const f32,
}
```

The queue family index field is filled with the value from before, and the queue count is hardcoded to 1. Queue priorities is an array of float values between 0.0 to 1.0, and allow you to prioritise certain queues, I'm only using 1 queue so I've hardcoded the value to `[1.0]`.

```pretty-rs
 let queue_create_infos: Vec<vk::DeviceQueueCreateInfo> = unique_queue_family_indices
    .iter()
    .map(|index| {
	vk::DeviceQueueCreateInfo::builder()
	    .queue_family_index((*index).try_into().unwrap())
	    .queue_priorities(&queue_priorities)
	    .build()
    })
    .collect();
```

I've ommitted some code where I turned the array of queue families into an array of unique family indexes, but this is how I'm dealing with the fact the graphics and presentation queue may be the same or different.

The last part of creating the `VkDevice` handle is the `VkDeviceFeatures` struct, which is a long list of boolean toggles that enable different optional features of the device, for now I'm leaving this blank but I will have to return to enable features as I go forward.

With that done, I finally wrote the code for creating the vulkan device:

```pretty-rs
let create_info = vk::DeviceCreateInfo::builder()
    .queue_create_infos(&queue_create_infos)
    .enabled_layer_names(&wanted_layers_raw)
    .enabled_extension_names(&wanted_extensions_raw)
    .enabled_features(&features);

let device = unsafe { instance.create_device(physical.physical, &create_info, None)? };
```

The queue handles have secretly been made by the API, but they're not returned by the `create_device` method, so I have to get them manually:

```pretty-rs
let graphics = device.get_device_queue(graphics_family_index.try_into().unwrap(), 0);
let graphics = Queue::new(graphics, graphics_family_index.try_into().unwrap());

let present = device.get_device_queue(present_family_index.try_into().unwrap(), 0);
let present = Queue::new(present, present_family_index.try_into().unwrap());
```

I then wrapped the `VkDevice` handle and the two queues into one `Device` struct that will be passed around the rest of my application:

```pretty-rs
pub struct Queues {
    pub graphics: Queue,
    pub present: Queue,
}

pub struct Device {
    device: ash::Device,
    physical: super::instance::PhysicalDevice,
    queues: Queues,
}

impl Device {
	pub unsafe fn new(instance: &Instance, surface: &Surface) -> Result<Self, vk::Result> {
		...

		Ok(Self {
		    device,
		    physical,
		    queues: Queues { graphics, present },
		})
	}
}
```

And with the device created, the next step is to create the swapchain, which is the next step in presenting images to the screen.

=== Swapchain

Now I have a surface representing the window I need a way of pushing rendered images for users to see, in Vulkan this done by transferring ownership of images off to the presentation system. The easiest approach to this might be to create an image at the start of a frame, render the scene to it and then transfer it off to the presentation system, however this has some downsides mainly the memory usage of creating an image every frame. A solution to this would be to share ownership of the image with the presentation system, however this means the game could be in the middle of rendering an image while its being presented to a window, which is the cause of screen tearing. 

The solution is a technique called double buffering, where you create two images at the start of the application and swap the ownership between the game and the presentation system. More than two images can be used in practice, and the array of images is called the swapchain, swap due to the ownership swapping and chain because of the arqray-like nature of it. Here's the rough pseudocode explaining the ownership transfer during a frame:

```pretty-rs
fn render_frame() {
  let frame = acquire_frame(); // Get a frame from the swapchain that isn't being held by the presentation layer
  render_scene(&frame); // Use that frame as a target for the render pass
  present(frame); // Return the frame to the presentation layer
}
```

This also has a slight issue that the game won't do anything while waiting for an image from the presentation system, meaning the tick rate will be tied to the frame rate unless I move it to a separate thread, but this won't be an issue for my game as the server will responsible for ticking the game world.

Double buffering is handled within vulkan but I'll need to consider it when getting frames later into rendering.

I'll start by encapsulating the swapchain into a `Swapchain` struct, as well as the vulkan swapchain it stores the format and extent of the images within the swapchain, as I'll need these later.

```pretty-rs
pub struct Swapchain {
    pub(crate) swapchain: vk::SwapchainKHR,
    pub format: vk::Format,
    pub extent: vk::Extent2D,
}
```

Then I'll start on the code to create the swapchain. To being I fetch some information about the surface from the surface extension and store them in variables for later processing: 

```pretty-rs
impl Swapchain {
    pub fn new(
        instance: &Instance,
        surface: &Surface,
        device: &Device,
        window: &Window,
    ) -> Result<Self, vk::Result> {
        let surface_khr = instance.extensions.surface.as_ref().unwrap(); // Get the surface extension object from the instance

        // General structure of surface properties
        let capabilities = unsafe {
            surface_khr
                .get_physical_device_surface_capabilities(device.physical.physical, surface.surface)
                .unwrap()
        };

        // List of available formats for the images in the swapchain
        let formats = unsafe {
            surface_khr
                .get_physical_device_surface_formats(device.physical.physical, surface.surface)
                .unwrap()
        };

        // List of available present modes, which include double buffering and some alternatives 
        let present_modes = unsafe {
            surface_khr
                .get_physical_device_surface_present_modes(
                    device.physical.physical,
                    surface.surface,
                )
                .unwrap()
        };

        ...
    }
}
```

Now we have the properties of the surface, I need to choose the format, present mode and resolution of the swapchain images:

```pretty-rs
impl Swapchain {
    pub fn new(
        instance: &Instance,
        surface: &Surface,
        device: &Device,
        window: &Window,
    ) -> Result<Self, vk::Result> {
        ...

        let format = formats
            .iter() // Iterate through formats
            .find(|format| {
                format.format == vk::Format::B8G8R8A8_SRGB // Prioritise a BGRA 32-bit colour image
                    && format.color_space == vk::ColorSpaceKHR::SRGB_NONLINEAR // Prioritise the SRGB colour scale
            })
            .unwrap_or(formats.first().unwrap()); // Default to the first available format

        let present_mode = present_modes
            .iter() // Iterator through available present modes
            .copied()
            .find(|present_mode| *present_mode == vk::PresentModeKHR::MAILBOX) // Prioritise triple buffering (like double buffer but without the frame rate lock)
            .unwrap_or(vk::PresentModeKHR::FIFO); // Default to double buffering

        // The extent of the swapchain is the resolution of images it contains
        let extent = if capabilities.current_extent.width != u32::MAX { // There's a special case where if the width is the max value of a 32-bit unsigned integer, the extent width is unrestricted
            capabilities.current_extent // If its not unrestricted, return whatever it thinks it should be
        } else {
            vk::Extent2D { // If it is restricted, return the size of the window
                width: window.inner_size().width,
                height: window.inner_size().height,
            }
        };

        let image_count = if capabilities.max_image_count == 0 // If the max_image_count is 0, the max is unrestricted
            || capabilities.min_image_count + 1 < capabilities.max_image_count
        {
            capabilities.min_image_count + 1 // Allocating a spare image can help to reduce the frame locking issue
        } else {
            capabilities.min_image_count // If the extra 1 image isn't available, just use the minimum
        };

        // If the present queue and the graphics queue are the same, then the images won't need to be shared between two queues
        let (sharing_mode, queue_family_indices) =
            if device.queues.graphics.index == device.queues.present.index { // If they are the same queue
                (vk::SharingMode::EXCLUSIVE, Vec::new()) // The ownership is exclusive
            } else {
                (
                    vk::SharingMode::CONCURRENT, // Else its shared between the present queue and the graphics queue
                    vec![device.queues.graphics.index, device.queues.present.index],
                )
            };

        ...
    }
}
```

With all the decisions about the swapchain made, its finally time to move onto the creation:

```pretty-rs
impl Swapchain {
    pub fn new(
        instance: &Instance,
        surface: &Surface,
        device: &Device,
        window: &Window,
    ) -> Result<Self, vk::Result> {
        ...

        // Use the builder pattern to fill out the create info
        let create_info = vk::SwapchainCreateInfoKHR::builder()
            .surface(surface.surface)
            .min_image_count(image_count)
            .image_format(format.format)
            .image_color_space(format.color_space)
            .image_extent(extent)
            .image_array_layers(1)
            .image_usage(vk::ImageUsageFlags::COLOR_ATTACHMENT)
            .image_sharing_mode(sharing_mode)
            .queue_family_indices(&queue_family_indices)
            .pre_transform(capabilities.current_transform)
            .composite_alpha(vk::CompositeAlphaFlagsKHR::OPAQUE)
            .present_mode(present_mode)
            .clipped(true);

        // Create the swapchain
        let swapchain = unsafe {
            device
                .extensions
                .swapchain
                .as_ref()
                .unwrap()
                .create_swapchain(&create_info, None)
                .unwrap()
        };

        // Return the encapsulated swapchain and store the format and extent for later use
        Ok(Self {
            swapchain,
            format: format.format,
            extent,
        })
    }
}
```