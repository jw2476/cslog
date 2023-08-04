== 2023-08-02 - Client Server Representation

With gathering and a basic inventory implemented on the client-side, I ran a brief playtest sending an executable and IP address to a few friends who are interested in the project. The resounding feedback was that were was no persistence, if they logged off and logged back in, their items would have been lost, this is because at the moment the server has no concept of an inventory or gathering, I was planning to move on to crafting and revisit this later but with the feedback recieved I decided to prioritise this issue. 

The first issue was going to be where to store the game data, below is a table evaluating the 
#table(
    columns: (auto, auto, auto),
    inset: 10pt,
    align: horizon,
    [*Option*], [*Advantages*], [*Disadvantages*],
    [Client-side storage],
    [
        - Minimises server load
        - No need to sync data with server
    ],
    [
        - Easy to tamper with data
        - Client is not always online
    ],
    [Server-side storage],
    [
        - Very difficult to tamper with data
        - Data is all in one place
        - Data is always accessible
    ],
    [
        - Increased server load
        - Clients need to tell server about all data changes
    ]
)

Since this system would be used to store inventory data it must be tamper-proof, otherwise it would be too easy for players to cheat items into the game, and any sort of economy would be impossible. Because of this I decided the server would have to store the data in some sort of central database. The server load issue this creates can be minimised using client prediction which I'll get to later in this devlog.


The choice to store data on the server then presents another issue, the same entity will be represented differently on the client and server, for example on a client, an item stack may be represented as having a type, quantity and then a resource handle to its icon texture for rendering to the screen, maybe also a handle to a 3d mesh of the item. On the server this same entity would just have a type and quantity, and maybe an player reference to who owns it. This is the kind of code that I started with for this example:

```pretty-rs
enum ItemKind { ... }

struct ServerItemStack {
    kind: ItemKind,
    quantity: u32,
    owner: &Player
}

struct ClientItemStack {
    kind: ItemKind,
    quantity: u32,
    icon: &Texture,
    model: &Mesh
}
```
There's a lot of shared code here and say I changed the quantity from the 4-byte u32 to the 8-byte u64 and forgot to change the other, any sort of decoding and encoding would break and that could cost hours in debugging time. I'd like the client and server to share as much code as possible to save time and make the software easier to change in future so I need some way to represent two permutations of the same entity while keeping shared bits. My first idea would be to use rust's `Option<T>` type and set the server specific fields to None on the client and vice-versa, the code would look like this:

```pretty-rs
enum ItemKind { ... }

struct ItemStack {
    kind: ItemKind,
    quantity: u32,
    owner: Option<&Player>,
    icon: Option<&Texture>,
    model: Option<&Mesh>
}
```

Now while this is a lot better, there's only one struct now, it's still quite clunky, the server and clients are given access to data that will never exist, this has a memory cost and a performance cost for passing around the struct, in addition to access the data is now hidden behind an unwrap, which has to be handled nicely on the server due to the crashing risk. Luckily rust has a feature called... features. They allow the programmer a to specify list of features for a crate that the user can toggle, and then code can be conditionally compiled based on the set features, a bit like C's \#ifdef. Rewriting the code above using features would look like:

```pretty-rs 
enum ItemKind { ... }

struct ItemStack {
    kind: ItemKind,
    quantity: u32,
    #[cfg(feature = "server")]
    owner: &Player,
    #[cfg(feature = "client")]
    icon: &Texture,
    #[cfg(feature = "client")]
    model: &Mesh
}
```

With this code, if the `client` feature is set, the struct `ItemStack` will be identical to the original `ClientItemStack`, and the same for the `server` feature and `ServerItemStack`. There are no options so no risk of crashing due to `unwrap` and no unessessary memory overhead as the fields don't exist where they don't need to! The `cfg` attribute macro also works for method definitions, so client and server can have different methods under the same name. With this designed I moved all data and entities into a crate named `common`.
