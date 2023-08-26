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

The choice to store data on the server then presents another issue, the same objects will often be represented differently on the client and server, for example here the client and server `Player` struct:

```pretty-rs 
struct Player {
    player: RenderObject,
    jump_t: f32,
    light: Light
}
```

```pretty-rs
struct Player {
    position: Vec3,
    username: String,
    inventory: Inventory,
}
```

The data is completely different, the server `Player` contains a username and inventory, whereas on the client these are separate systems. This is an example of a struct that should be kept separate between server and client, however there are structs with more in common between server and client such as the `Item` enum which contains a list of all items in the game, at this point I moved these more common data structures into a common crate which can be used by both the client and server crates.

So far the network protocol had been manually encoding data into bytes which lead to a lot of bugs and errors along the way. Luckily someone has made an amazing crate called `serde` which allows automatic encoding and decoding of data structures. In addition to `serde`, I needed a crate that encodes data into a tightly packet binary stream, I took a while to look through the option and it seemed like `postcard` was the best option. Refactoring to use `serde` and `postcard` took the network code from

```pretty-rs 
#[derive(FromPrimitive, ToPrimitive)]
pub enum ServerboundOpcode {
    Login,
    Move,
    Heartbeat,
    Disconnect,
}

pub struct ServerboundPacket {
    pub opcode: ServerboundOpcode,
    pub payload: Vec<u8>,
}

#[derive(FromPrimitive, ToPrimitive)]
pub enum ClientboundOpcode {
    SpawnPlayer,
    Move,
    DespawnPlayer,
    NotifyDisconnection,
    Kick,
}

pub struct ClientboundPacket {
    pub opcode: ClientboundOpcode,
    pub payload: Vec<u8>,
}

impl ClientboundPacket {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut data = Vec::new();
        data.extend(&self.opcode.to_u32().unwrap().to_be_bytes());
        data.extend(&self.payload.clone());
        let mut packet = data.len().to_be_bytes().to_vec();
        packet.append(&mut data);
        packet
    }
}

impl ServerboundPacket {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut data = Vec::new();
        data.extend(&self.opcode.to_u32().unwrap().to_be_bytes());
        data.extend(&self.payload.clone());
        let mut packet = data.len().to_be_bytes().to_vec();
        packet.append(&mut data);
        packet
    }
}
```

to

```pretty-rs
pub mod server {
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct Login {
        pub username: String,
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct Move {
        pub position: glam::Vec3,
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub enum Packet {
        Login(Login),
        Move(Move),
        Heartbeat,
        Disconnect,
    }
}

pub mod client {
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct SpawnPlayer {
        pub username: String,
        pub position: glam::Vec3,
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct DespawnPlayer {
        pub username: String,
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct Move {
        pub username: String,
        pub position: glam::Vec3,
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct NotifyDisconnection {
        pub reason: String,
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub enum Packet {
        SpawnPlayer(SpawnPlayer),
        DespawnPlayer(DespawnPlayer),
        Move(Move),
        NotifyDisconnection(NotifyDisconnection),
    }
}
```

Here's an example of where it simplified the most code, this is server side decoding before and after this change:

```pretty-rs
let packet_size =
    if let Some(array) = buf.get(0..8).and_then(|bytes| bytes.try_into().ok()) {
        u64::from_be_bytes(array)
    } else {
        warn!("Failed to read packet due to underflow");
        continue;
    };

let Some(packet) = buf.get(8..(packet_size as usize + 8)) else {
    warn!("Failed to read packet due to underflow");
    continue
};

let opcode =
    if let Some(array) = packet.get(0..4).and_then(|bytes| bytes.try_into().ok()) {
        u32::from_be_bytes(array)
    } else {
        warn!("Packet of size {} is too short", packet.len());
        continue;
    };

let Some(opcode) = ServerboundOpcode::from_u32(opcode) else {
    warn!("Invalid opcode: {}", opcode);
    continue
};

let Some(payload) = packet.get(4..).map(<[u8]>::to_vec) else {
   warn!("Failed to read packet body");
   continue
};

let packet = ServerboundPacket { opcode, payload };
```

to just:

```pretty-rs
let packet = match postcard::from_bytes(&buf) {
    Ok(packet) => packet,
    Err(e) => {
        warn!("Failed to decode packet due to {}", e);
        continue;
    }
};

if let Err(e) = handle_packet(&mut server, &packet, addr) {
    warn!("Handling packet failed with {e}");
    continue;
}
```

It also made using the packet payloads a lot easier since the handlers don't have to worry about decoding the binary, for example:

```pretty-rs
let username = match String::from_utf8(packet.payload.clone()) {
    Ok(str) => str.trim().to_owned(),
    Err(e) => {
        warn!("Failed to parse username: {}", e);
        if let Err(e) = disconnect(server, addr, Some("Invalid username".to_owned())) {
            warn!("Failed to disconnect client due to {}", e);
        }
        return;
    }
};
let client = Client {
    username,
    player_translation: Vec3::new(0.0, 0.0, 0.0),
    last_heartbeat: Instant::now(),
};
```

became just:

```pretty-rs
let client = Client {
    username: packet.username.clone(),
    player_translation: Vec3::new(0.0, 0.0, 0.0),
    last_heartbeat: Instant::now(),
};
```

While testing all server functionality I found the player would jitter when moving and peers positions would not update, looking at client logs when client A would move, client A would recieve a packet updating its position, client B wouldn't. Must be a server bug, issue was when sending the movement update packets, taking a look at the server-side `handle_move` function:

```pretty-rs
fn handle_move(server: &mut Server, packet: &net::server::Move, addr: SocketAddr) {
    // Find client, early returning if not found
    let Some(client) = server
        .connections
        .get_mut(&addr) else {
            warn!("Cannot find client for addr {}", addr);
            return;
        };

    client.player_translation = packet.position;

    info!(
        "Updated position for {} to {:?}",
        client.username, client.player_translation
    );

    // Have to refetch to drop the mutable reference
    let Some(client) = server
        .connections
        .get(&addr) else {
            warn!("Cannot find client for addr {}", addr);
            return;
        };

    for peer_addr in server.connections.keys() {
        if *peer_addr == addr { // Don't want to send to itself, will cause jitter
            continue;
        }

        // Prepare packet
        let packet = net::client::Packet::Move(net::client::Move {
            username: client.username.clone(),
            position: client.player_translation
        });

        if let Err(e) = server.send(addr, &packet) {
            warn!(
                "Failed to notify {} of {} moving due to {}",
                peer_addr, client.username, e
            );
            continue;
        }
    }
}
```

The issue was with the destination address of the packet, I was sending them to `addr` which was the address of the client that sent the packet. I should have been sending them to the peer's address, stored in `peer_addr`, here are the changes I made to fix it:

```pretty-rs
if let Err(e) = server.send(addr, &packet) {
    warn!(
        "Failed to notify {} of {} moving due to {}",
        peer_addr, client.username, e
    );
    continue;
}
```

needed to be:

```pretty-rs
if let Err(e) = server.send(*peer_addr, &packet) {
    warn!(
        "Failed to notify {} of {} moving due to {}",
        peer_addr, client.username, e
    );
    continue;
}
```

The dereference was needed as the `peer_addr` was borrowed from the hashmap of clients. 

After testing again I found another issue, the first client would connect fine, second client would connect and the first would crash due to 'Unknown peer', this meant there was an issue with the `handle_login` function and it wasn't notifying peers about a new connection.

```pretty-rs
fn handle_login(server: &mut Server, packet: &net::server::Login, addr: SocketAddr) {
    let client = Client {
        username: packet.username.clone(),
        player_translation: Vec3::new(0.0, 0.0, 0.0),
        last_heartbeat: Instant::now(),
    };

    // Notify peers about new client
    for peer_addr in server.connections.keys() {
        let packet = net::client::Packet::SpawnPlayer(net::client::SpawnPlayer {
            username: client.username.clone(),
            position: client.player_translation,
        });
        if let Err(e) = server.send(addr, &packet) {
            warn!("Failed to notify {} of new player due to {}", peer_addr, e);
        }
    }

    // Notify client about existing peers
    for (peer_addr, peer_client) in &server.connections {
        let packet = net::client::Packet::SpawnPlayer(net::client::SpawnPlayer {
            username: peer_client.username.clone(),
            position: peer_client.player_translation,
        });
        if let Err(e) = server.send(addr, &packet) {
            warn!(
                "Failed to notify new player {} of player {} due to {}",
                addr, peer_addr, e
            );
        }
    }

    info!("Added {} to connection list", client.username);
    server.connections.insert(addr, client);
}
```

Taking a look it was the same bug as before, now on line 14, the first client was complaining about unknown peers because it wasn't being notified, the newly connected client was as `addr` was being used instead of `peer_addr`. This can be fixed with a quick change:

```pretty-rs
// Notify peers about new client
for peer_addr in server.connections.keys() {
    let packet = net::client::Packet::SpawnPlayer(net::client::SpawnPlayer {
        username: client.username.clone(),
        position: client.player_translation,
    });
    if let Err(e) = server.send(addr, &packet) {
        warn!("Failed to notify {} of new player due to {}", peer_addr, e);
    }
}
```

should have been:

```pretty-rs
// Notify peers about new client
for peer_addr in server.connections.keys() {
    let packet = net::client::Packet::SpawnPlayer(net::client::SpawnPlayer {
        username: client.username.clone(),
        position: client.player_translation,
    });
    if let Err(e) = server.send(*peer_addr, &packet) {
        warn!("Failed to notify {} of new player due to {}", peer_addr, e);
    }
}
```

And with that everything was working again, now the server and client both had access to the item data and networking was refactored I finally moved onto inventory syncing. I chose the simplest approach, the client notifes the server about a new item, this approach has a few issues, mainly that it would be very easy to spawn items in since there can be no validation. Eventually I'd want to make the client tell the server about an action such as gathering or crafting, and then the server validates the action and calculates the resulting effects such as modifying the player's inventory, and then notifies the client of the effects, however since the server currently has no knowledge of the game world, and I said before there's little point until I start working on a static/procedurally generated world, it can't validate these actions so this simple approach is all that's possible. 

Firstly I designed the packet `ModifyInventory`, it needed to both add and remove item stacks (I chose item stacks over indiviual items to reduce the number of packets being sent to the server), the struct for the packet looked like this, I could simplify adding and removing stacks by just making the server respond by setting the stack quantity to the amount in the packet. So if the client sent a packet with a stack containing `(Wood, 4)`, the server would set that player's quantity of wood to 4. If its set to 0, its treated as deleting the stack. 

```pretty-rs
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ModifyInventory {
    stack: ItemStack
}
```

Now I had to hook into `Inventory` struct's `add` method, and whenever it is called dispatch a `ModifyInventory` packet to the server. With the current system with `Inventory` in the common crate, this was very difficult as whatever changes were made would affect both the server and client, I needed a way of specifying behaviour specific to the client or the server, so far I had only done this with data. Here are a couple solutions I came up with and evalutated:

#table(
    columns: (auto, auto, auto),
    inset: 10pt,
    align: horizon,
    [*Option*], [*Advantages*], [*Disadvantages*],
    [Inject middleware closures that are called when the method is called],
    [
        - Function signatures remain the same as the closures will capture any side specific state
        - Adding middleware capabilities could be automated with a macro
    ],
    [
        - Adds a layer of indirection when you call a function, unknown side-effects, could lead to accidental deadlocks or performance issues
        - Makes constructors a lot more complicated
    ],
    [Move the struct from the common crate into seperate server and client structs],
    [
        - Easy to do
        - No complex constructors
    ],
    [
        - Data and code could be duplicated, possibly causing a data desync
        - Common crate has no knowledge of this system
    ],
    [Swap the struct for a trait which then the side specific structs implement aka dependency injection],
    [
        - Useful if the common crate needs to use the system as it can be passed as a `&dyn Trait`
        - Easy to do
    ],
    [
        - Data and code could be duplicated, possibly causing a data desync, however this can be reduced using default implementations
        - Boilerplate
    ],
    [Use features to disable certain functions],
    [
        - Easy to do
        - Useful if the common crate needs to use the system as it can be passed as a `&dyn Trait`
    ],
    [
        - Code has to be written in the common crate, so it can't use any client/server specific bits
    ],
    [Wrap the struct with a new struct in the side specific crates],
    [
        - Allows for the use of code in side specific crates
    ],
    [
        - Common crate can't use the specialised methods
        - Results in a bit of boilerplate
    ],
    [Observables],
    [
        - Observers can be registered from either side to watch the changes and respond
    ],
    [
        - Can add a little boilerplate
    ]
)

I don't think there's a one size fits all solution to this senario, but in the case of the Inventory struct I decided to keep the `Inventory` struct split and have different structs and code on the client and server sides, I was leaning towards observables but working out whats changed can be very complicated and easy to introduce bugs. The client side inventory needed to store a reference to the socket so it could send packets, so the struct looked like:

```pretty-rs
use common::ItemStack;

struct Inventory {
    inventory: Vec<ItemStack>,
    socket: Arc<Socket>
}
```

and on the server to socket wasn't needed so the struct looked like:

```pretty-rs
use common::ItemStack;

struct Inventory {
    inventory: Vec<ItemStack>
}
```

I may choose to replace the `Vec<ItemStack>` with some sort of indexed map in future to bring lookups from O(n) to O(1), but for now the vector is fine. With that done, I moved onto implementation, the server implementation was just the same as the existing client implementation so I copied that over before modifying the client implementation to include sockets.

The only changes needed for the client was taking the socket reference in the constructor and sending the packet when either `add` or `set` were called. I chose to add a new private method called `update` that sent the ModifyInventory packet to the server.

```pretty-rs
impl Inventory {
    pub fn new(socket: Arc<Socket>) -> Self {
        Self {
            inventory: Vec::new(),
            socket
        }
    }

    fn update(&self, item: Item) {
        let Some(stack) = self.inventory.iter().find(|s| s.item == item) else {
            warn!("Tried to update stack {:?} that doesn't exist", item);
            return;
        };


        let packet = net::server::Packet::ModifyInventory(net::server::ModifyInventory {
            stack: stack.clone()
        });
        if let Err(e) = self.socket.send(&packet) {
            warn!("Failed to update stack {:?} due to {}", item, e);
            return;
        }
    }

    pub fn add(&mut self, stack: ItemStack) {
        if let Some(existing) = self.inventory.iter_mut().find(|s| s.item == stack.item) {
            existing.amount += stack.amount;
        } else {
            self.inventory.push(stack);
        }

        self.update(stack.item);
    }

    pub fn set(&mut self, stack: ItemStack) {
        if let Some(existing) = self.inventory.iter_mut().find(|s| s.item == stack.item) {
            existing.amount = stack.amount;
        } else {
            self.inventory.push(stack);
        }
        
        self.update(stack.item);
    }

    pub fn get_items(&self) -> &[ItemStack] {
        &self.inventory
    }
}
```

I handled the socket errors in the update function which is bad practice but I didn't want to be bubbling up Results everywhere, eventually there may be a wrapper around the socket that attempts to retry sending packets to recover from temporary losses of connection but that's a problem to solve in the future

With ModifyInventory packets being sent to the server, I now to handle them by updating the server-side inventory for the player. Firstly I updated the `Connection` struct to include an inventory:

```pretty-rs
struct Connection {
    player_translation: Vec3,
    username: String,
    last_heartbeat: Instant,
    inventory: Inventory // New
}
```

Next I needed the logic to update the item stacks on a `ModifyInventory` packet, I put this code in a new function called `handle_modify_inventory`:

```pretty-rs
fn handle_modify_inventory(server: &mut Server, packet: &net::server::ModifyInventory, addr: SocketAddr) {
    // Get connection
    let Some(connection) = server
        .connections
        .get_mut(&addr) else {
            warn!("Cannot find connection for addr {}", addr);
            return;
        };

    // Update inventory
    connection.inventory.set(packet.stack);
}
```

Now the server was keeping track of the players' inventories, however when the connection was lost, the `Connection` structs were dropped out of the HashMap, loosing the inventories in the process. I needed to store player data separately from connection info so the data can be reused by multiple connections. I came up with a couple ways of doing this:

#table(
    columns: (auto, auto, auto),
    inset: 10pt,
    align: horizon,
    [*Option*], [*Advantages*], [*Disadvantages*],
    [Store a list of players and a list of connections, with each connection storing the player's username so the player can be searched for],
    [
        - Simple to store
    ],
    [
        - Lots of Options to handle due to searching for players
    ],
    [Store a list of players and a list of connections, with each connection having a reference to a player],
    [
        - Easy to index
    ],
    [
        - Raw references result in lifetime nightmare
        - Shared ownership i.e. Mutex, RwLock needs locking, lots of Results to handle
    ],
    [Store a list of offline players, and a list of online connection, on login the player is moved out of the list and into a connection object which is put into the online connections list, undo on disconnect],
    [
        - No references, so no Results
        - No searches, so no Options
    ],
    [
        - Puts a bit more logic into connection and disconnection, need to be careful not to drop player info
    ]
)

I decided to go with storing a list of offline players and a list of online connections because it didn't need any error handling so the code would be simpler. At this point I decided to implement that indexed map I talked about earlier, its a wrapper around a HashMap where the keys can be derived from the value, for players that's their username (I'll probably replace this with UUIDs at some point), for connections that's their socket address, here's the code for the IndexedMap and the Unique trait which I use to implement how to go from the value to the key:

```pretty-rs
trait Unique {
    type Key: Eq + PartialEq + Hash;

    fn get_unique_key(&self) -> Self::Key;
}

struct IndexedMap<T>
where
    T: Unique,
{
    inner: HashMap<T::Key, T>,
}

impl<T> IndexedMap<T>
where
    T: Unique + Clone,
{
    fn new() -> Self {
        Self::default()
    }

    fn get(&self, key: &T::Key) -> Option<&T> {
        self.inner.get(key)
    }

    fn get_mut(&mut self, key: &T::Key) -> Option<&mut T> {
        self.inner.get_mut(key)
    }

    fn insert(&mut self, value: T) {
        self.inner.insert(value.get_unique_key(), value);
    }

    fn remove(&mut self, key: &T::Key) {
        self.inner.remove(key);
    }

    fn values<'a>(&'a self) -> Values<'a, T::Key, T> {
        self.inner.values()
    }

    fn keys<'a>(&'a self) -> Keys<'a, T::Key, T> {
        self.inner.keys()
    }

    fn take(&mut self, key: &T::Key) -> Option<T> {
        let value = self.get(key).cloned();
        self.remove(key);
        value
    }
}

impl<T> Default for IndexedMap<T>
where
    T: Unique,
{
    fn default() -> Self {
        Self {
            inner: HashMap::new(),
        }
    }
}
```

With the IndexedMap implemented I moved onto updating the Player, Connection and Server structs to fit the specification above:

```pretty-rs
struct Player {
    position: Vec3,
    inventory: Inventory
}

struct Connection {
    last_heartbeat: Instant,
    addr: SocketAddr,
    player: Player
}

struct Server {
    socket: UdpSocket,
    offline: IndexedMap<Player>,
    online: IndexedMap<Connection>
}
```

For IndexedMap to work with players and connections, they need to implement the Unique trait which I've done below:

```pretty-rs
impl Unique for Player {
    type Key = String;

    fn get_unique_key(&self) -> Self::Key {
        self.username.clone()
    }
}

impl Unique for Connection {
    type Key = SocketAddr;

    fn get_unique_key(&self) -> Self::Key {
        self.addr
    }
}
```

I also implemented Deref and DerefMut for Connection so the Player instance within can be easily accessed:

```pretty-rs
impl Deref for Connection {
    type Target = Player;

    fn deref(&self) -> &Self::Target {
        &self.player
    }
}

impl DerefMut for Connection {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.player
    }
}
```

Next was to change `handle_login` to take the player out of the offline map, wrap it in a Connection and insert it into the online map

```pretty-rs
fn handle_login(server: &mut Server, packet: &net::server::Login, addr: SocketAddr) {
    let player = server.offline.take(&packet.username).unwrap_or(Player {
        position: Vec3::ZERO,
        username: packet.username.clone(),
        inventory: Inventory::new()
    });

    server.online.insert(Connection { last_heartbeat: Instant::now(), addr, player });
    
    let connection = server.online.get(&addr).expect("Failed to get connection that was just inserted, this is very bad");

    ...
```

and then do the opposite when the client disconnects:

```pretty-rs
fn disconnect(server: &mut Server, addr: SocketAddr, reason: Option<String>) -> Result<()> {
    let Some(connection) = server.online
        .get(&addr) else {
            warn!("Cannot find client for addr {}", addr);
            return Err(std::io::Error::new(std::io::ErrorKind::NotFound, "Client not found").into());
        };
    info!("{} is disconnecting", connection.player.username);

    ... // Peers are notified, if it was a forced disconnection, reason is sent to the client

    server.offline.insert(connection.player.clone());
    server.online.remove(&connection.get_unique_key());
}
```

At this point the server was running and I did some testing, I logged on, harvested some trees, login out and back in again and my items didn't reappear, but in the server logs my items were found from my previous session, I wasn't setting the client's inventory when they logged in. I repurposed the ModifyInventory packet and made it common to both the server and client protocols, then on the server I needed to send a ModifyInventory packet for every item stack in that player's inventory, and on the client I needed to handle these packets by setting those item stacks to the inventory.

```pretty-rs
fn handle_login(server: &mut Server, packet: &net::server::Login, addr: SocketAddr) {
    ... // Login and insert connection

    for stack in connection.player.inventory.get_items() {
        let packet =
            net::client::Packet::ModifyInventory(net::client::ModifyInventory { stack: *stack });

        if let Err(e) = server.send(connection, &packet) {
            warn!(
                "Failed to update player {}'s inventory stack {:?} due to {}",
                connection.player.username, stack, e
            );
            continue;
        }

        info!(
            "Updating player {}'s stack {:?}",
            connection.player.username, stack
        );
    }
}
```

and the client-side is as simple as calling `Inventory::set` in the network handling function:

```pretty-rs
...

net::client::Packet::ModifyInventory(packet) => {
    info!("Setting {:?} to {}", packet.stack.item, packet.stack.amount);
    inventory.set(packet.stack);
}

...
```

And with that I did another round of testing, this was a big server refactor, so I did a full networking test trying to break the server by making it handle lots of clients at once, and everything passed. Inventories were persisting through sessions, and were being updated nearly instantly on login. With this set of features done I did a round of testing with my friends over the internet, to test latency and different computers, and also to gather some feedback before deciding what to implement next.