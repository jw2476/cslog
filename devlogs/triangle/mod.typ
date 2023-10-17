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

Before I get to logical devices, I needed to do some research into Vulkan's execution model so I understood what queues were. All operations in Vulkan have to be recorded into `CommandBuffer` objects, from drawing to rescaling an image. Once these `CommandBuffer` objects are recorded they can be submitted to a `Queue` on the GPU, `Queue`s are a bit like threads but on a GPU, each `Queue` can have a `CommandBuffer` submitted, so multiple `CommandBuffer`s can be executed at once. For this program I am only using one `Queue` so the graphics code will be executed like a single-threaded program, I decided this because of the extra complexity multi-threaded GPU code would create. Each `Queue` belongs to a queue family, which is a collection of `Queue`s with the same capabilities, such as presenting, or running compute shaders. Using the CPU analogy, `Queue`s would be the threads and queue families would be the cores. 

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


