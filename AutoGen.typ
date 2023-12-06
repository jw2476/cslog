
= Appendix: Code
#[
    #set align(center)
    The following is every single `.rs` (Rust) and `.toml` (configuration) file in my project. The total length is 7939 loc and 224 548 characters across 68 files.
]

=== arbiter::main.rs

```pretty-rs
// main.rs
#![warn(clippy::pedantic)]
#![warn(clippy::nursery)]
#![deny(clippy::unwrap_used)]
#![warn(clippy::expect_used)]

use anyhow::Result;
use async_std::net::UdpSocket;
use common::{
    item::{Item, ItemStack},
    net,
};
use glam::Vec3;
use num_traits::{FromPrimitive, ToPrimitive};
use sqlx::SqlitePool;
use std::{
    collections::{
        hash_map::{Keys, Values},
        HashMap,
    },
    hash::Hash,
    net::SocketAddr,
    ops::Deref,
    time::Instant,
};
use tracing::{error, info, warn};

#[derive(Clone, PartialEq, Eq)]
struct Connection {
    last_heartbeat: Instant,
    addr: SocketAddr,
    user_id: i64,
    character_id: i64,
}

trait Unique {
    type Key: Eq + PartialEq + Hash;

    fn get_unique_key(&self) -> Self::Key;
}

impl Unique for Connection {
    type Key = SocketAddr;

    fn get_unique_key(&self) -> Self::Key {
        self.addr
    }
}

impl Deref for Connection {
    type Target = SocketAddr;

    fn deref(&self) -> &Self::Target {
        &self.addr
    }
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
    pub fn new() -> Self {
        Self::default()
    }

    pub fn get(&self, key: &T::Key) -> Option<&T> {
        self.inner.get(key)
    }

    pub fn get_mut(&mut self, key: &T::Key) -> Option<&mut T> {
        self.inner.get_mut(key)
    }

    pub fn insert(&mut self, value: T) {
        self.inner.insert(value.get_unique_key(), value);
    }

    pub fn remove(&mut self, key: &T::Key) {
        self.inner.remove(key);
    }

    pub fn values<'a>(&'a self) -> Values<'a, T::Key, T> {
        self.inner.values()
    }

    pub fn keys<'a>(&'a self) -> Keys<'a, T::Key, T> {
        self.inner.keys()
    }

    pub fn take(&mut self, key: &T::Key) -> Option<T> {
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

struct Server {
    socket: UdpSocket,
    online: IndexedMap<Connection>,
    pool: SqlitePool,
}

#[derive(thiserror::Error, Debug)]
enum SendError {
    #[error("IO error")]
    IOError(#[from] std::io::Error),
    #[error("Encode error")]
    EncodeError(#[from] postcard::Error),
}

impl Server {
    pub fn new(socket: UdpSocket, pool: SqlitePool) -> Self {
        Self {
            socket,
            online: IndexedMap::new(),
            pool,
        }
    }

    pub async fn send(
        &self,
        addr: &SocketAddr,
        packet: &net::client::Packet,
    ) -> Result<(), SendError> {
        let bytes = postcard::to_stdvec(packet)?;
        self.socket.send_to(&bytes, addr).await?;
        Ok(())
    }
}

#[async_std::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();

    let socket = UdpSocket::bind("0.0.0.0:8000").await?;

    let pool = SqlitePool::connect(&std::env::var("DATABASE_URL")?).await?;
    sqlx::migrate!().run(&mut pool.acquire().await?).await?;

    let mut server = Server::new(socket, pool);
    info!("Listening on 0.0.0.0:8000");

    let mut last_heartbeat_check = Instant::now();

    loop {
        let mut buf = [0; 4096];
        match server.socket.recv_from(&mut buf).await {
            Err(e) => panic!("{e}"),
            Ok((_, addr)) => {
                let packet = match postcard::from_bytes(&buf) {
                    Ok(packet) => packet,
                    Err(e) => {
                        warn!("Failed to decode packet due to {}", e);
                        continue;
                    }
                };

                if let Err(e) = handle_packet(&mut server, &packet, addr).await {
                    warn!("Handling packet failed with {e}");
                    continue;
                }
            }
        }

        if last_heartbeat_check.elapsed().as_secs_f32() > 1.0 {
            info!("Checking heartbeats");
            check_heartbeats(&mut server).await?;
            last_heartbeat_check = Instant::now();
        }
    }
}

async fn check_heartbeats(server: &mut Server) -> Result<()> {
    let dead = server
        .online
        .values()
        .filter(|connection| connection.last_heartbeat.elapsed().as_secs_f32() > 20.0)
        .map(|connection| connection.addr)
        .collect::<Vec<SocketAddr>>();

    for addr in dead {
        if let Err(e) = disconnect(server, addr, Some("Heartbeat timeout".to_owned())).await {
            warn!("Failed to disconnect {} due to {}", addr, e);
            continue;
        }
    }

    Ok(())
}

async fn handle_login(server: &mut Server, packet: &net::server::Login, addr: SocketAddr) {
    let Ok(user) = sqlx::query!(
        "SELECT id, password FROM users WHERE username = ?",
        packet.username
    )
    .fetch_optional(&server.pool)
    .await
    else {
        error!("Fetching user {} failed", packet.username);
        return;
    };

    let Some(user) = user else {
        let _ = send_error(server, addr, "No account found for that username", true).await;
        return;
    };

    if user.password != packet.password {
        let _ = send_error(server, addr, "Username or password is incorrect", true).await;
        return;
    }

    let Ok(character) = sqlx::query!(
        "SELECT id, name, position_x, position_y, position_z FROM characters WHERE owner = ?",
        user.id
    )
    .fetch_one(&server.pool)
    .await
    else {
        error!("Fetching character for user {} failed", packet.username);
        return;
    };
    let position = Vec3::new(
        character.position_x as f32,
        character.position_y as f32,
        character.position_z as f32,
    );

    server.online.insert(Connection {
        last_heartbeat: Instant::now(),
        addr,
        user_id: user.id,
        character_id: character.id,
    });

    let connection = server
        .online
        .get(&addr)
        .expect("Failed to get connection that was just inserted, this is very bad");

    for peer in server.online.values() {
        // Notify peers about new client
        let packet = net::client::Packet::SpawnPlayer(net::client::SpawnPlayer {
            username: packet.username.clone(),
            position,
        });

        if let Err(e) = server.send(peer, &packet).await {
            warn!("Failed to notify {} of new player due to {}", peer.addr, e);
        }

        let Ok(peer_user) = sqlx::query!("SELECT username FROM users WHERE id = ?", peer.user_id)
            .fetch_one(&server.pool)
            .await
        else {
            error!("Fetching peer user {} failed", peer.user_id);
            continue;
        };

        let Ok(peer_character) = sqlx::query!(
            "SELECT position_x, position_y, position_z FROM characters WHERE id = ?",
            peer.character_id
        )
        .fetch_one(&server.pool)
        .await
        else {
            error!("Fetching peer character {} failed", peer.character_id);
            continue;
        };

        let peer_position = Vec3::new(
            peer_character.position_x as f32,
            peer_character.position_y as f32,
            peer_character.position_z as f32,
        );

        // Notify new client about peers
        let packet = net::client::Packet::SpawnPlayer(net::client::SpawnPlayer {
            username: peer_user.username.clone(),
            position: peer_position,
        });

        if let Err(e) = server.send(connection, &packet).await {
            warn!(
                "Failed to notify new player {} of player {} due to {}",
                addr, peer.addr, e
            );
        }
    }

    let Ok(items) = sqlx::query!(
        "SELECT item, quantity FROM items WHERE owner = ?",
        character.id
    )
    .fetch_all(&server.pool)
    .await
    else {
        error!(
            "Fetching items for character {} user {} failed",
            character.name, packet.username
        );
        return;
    };

    // Set clients inventory
    for stack in items {
        let Some(item) = Item::from_i64(stack.item) else {
            error!("Invalid item ID in database {}", stack.item);
            continue;
        };

        let inventory_packet = net::client::Packet::ModifyInventory(net::client::ModifyInventory {
            stack: ItemStack {
                item,
                amount: stack.quantity as u32,
            },
        });

        if let Err(e) = server.send(connection, &inventory_packet).await {
            warn!(
                "Failed to update player {}'s inventory stack {:?} due to {}",
                packet.username, stack, e
            );
            continue;
        }

        info!("Updating player {}'s stack {:?}", packet.username, stack);
    }

    info!("Added {} to connection list", packet.username);
}

async fn handle_move(server: &mut Server, packet: &net::server::Move, addr: SocketAddr) {
    let Some(connection) = server.online.get_mut(&addr) else {
        warn!("Cannot find client for addr {}", addr);
        return;
    };

    if let Err(e) = sqlx::query!(
        "UPDATE characters SET position_x = ?, position_y = ?, position_z = ? WHERE id = ?",
        packet.position.x,
        packet.position.y,
        packet.position.z,
        connection.character_id
    )
    .execute(&server.pool)
    .await
    {
        error!(
            "Updating position for character {} failed due to {}",
            connection.character_id, e
        );
        return;
    }

    info!(
        "Updated position for {} to {:?}",
        connection.character_id, packet.position
    );

    let Some(connection) = server.online.get(&addr) else {
        warn!("Cannot find client for addr {}", addr);
        return;
    };

    let Ok(user) = sqlx::query!(
        "SELECT username FROM users WHERE id = ?",
        connection.user_id
    )
    .fetch_one(&server.pool)
    .await
    else {
        error!("Failed to fetch user with id {}", connection.user_id);
        return;
    };

    for peer in server.online.values().filter(|peer| peer != &connection) {
        let packet = net::client::Packet::Move(net::client::Move {
            username: user.username.clone(),
            position: packet.position,
        });

        if let Err(e) = server.send(peer, &packet).await {
            warn!(
                "Failed to notify {} of {} moving due to {}",
                peer.addr, user.username, e
            );
            continue;
        }
    }
}

fn handle_heartbeat(server: &mut Server, addr: SocketAddr) {
    let Some(connection) = server.online.get_mut(&addr) else {
        warn!("Cannot find client for addr {}", addr);
        return;
    };

    connection.last_heartbeat = Instant::now();
    info!("{} heartbeat", connection.user_id);
}

async fn handle_packet(
    server: &mut Server,
    packet: &net::server::Packet,
    addr: SocketAddr,
) -> Result<()> {
    match packet {
        net::server::Packet::Login(packet) => handle_login(server, packet, addr).await,
        net::server::Packet::Move(packet) => handle_move(server, packet, addr).await,
        net::server::Packet::Heartbeat => handle_heartbeat(server, addr),
        net::server::Packet::Disconnect => disconnect(server, addr, None).await?,
        net::server::Packet::ModifyInventory(packet) => {
            handle_modify_inventory(server, packet, addr).await
        }
        net::server::Packet::Signup(packet) => handle_signup(server, packet, addr).await,
    };

    Ok(())
}

async fn disconnect(server: &mut Server, addr: SocketAddr, reason: Option<String>) -> Result<()> {
    let Some(connection) = server.online.get(&addr) else {
        warn!("Cannot find client for addr {}", addr);
        return Err(std::io::Error::new(std::io::ErrorKind::NotFound, "Client not found").into());
    };
    info!("{} is disconnecting", connection.user_id);

    let user = sqlx::query!(
        "SELECT username FROM users WHERE id = ?",
        connection.user_id
    )
    .fetch_one(&server.pool)
    .await?;

    for peer in server.online.values().filter(|peer| peer != &connection) {
        let packet = net::client::Packet::DespawnPlayer(net::client::DespawnPlayer {
            username: user.username.clone(),
        });

        server.send(peer, &packet).await?;
    }

    if let Some(reason) = reason {
        let packet =
            net::client::Packet::NotifyDisconnection(net::client::NotifyDisconnection { reason });
        server.send(&connection, &packet).await?;
    }

    server.online.remove(&connection.get_unique_key());

    Ok(())
}

async fn send_error(server: &Server, addr: SocketAddr, message: &str, fatal: bool) {
    let packet = net::client::Packet::DisplayError(net::client::DisplayError {
        message: message.to_owned(),
        fatal,
    });
    let _ = server.send(&addr, &packet).await;
}

async fn handle_modify_inventory(
    server: &mut Server,
    packet: &net::server::ModifyInventory,
    addr: SocketAddr,
) {
    let Some(connection) = server.online.get_mut(&addr) else {
        warn!("Cannot find client for addr {}", addr);
        return;
    };

    let item = packet.stack.item.to_i64();

    let Ok(existing) = sqlx::query!(
        "SELECT id FROM items WHERE owner = ? AND item = ?",
        connection.character_id,
        item
    )
    .fetch_optional(&server.pool)
    .await
    else {
        error!(
            "Failed to fetch existing item stack character {} item {}",
            connection.character_id, packet.stack.item
        );
        return;
    };

    let result = if let Some(_) = existing {
        sqlx::query!("UPDATE items SET quantity = ?", packet.stack.amount)
            .execute(&server.pool)
            .await
    } else {
        sqlx::query!(
            "INSERT INTO items (item, quantity, owner) VALUES (?, ?, ?)",
            item,
            packet.stack.amount,
            connection.character_id
        )
        .execute(&server.pool)
        .await
    };

    if let Err(e) = result {
        error!(
            "Failed to set item stack {:?} for character {} due to {}",
            packet.stack, connection.character_id, e
        );
    }
}

async fn handle_signup(server: &Server, packet: &net::server::Signup, addr: SocketAddr) {
    let Ok(existing) = sqlx::query!("SELECT id FROM users WHERE username = ?", packet.username)
        .fetch_optional(&server.pool)
        .await
    else {
        error!("Failed to fetch existing user for signup");
        send_error(server, addr, "Server error", true).await;
        return;
    };

    if let Some(_) = existing {
        send_error(server, addr, "User exists with that username", true).await;
        return;
    }

    if let Err(e) = sqlx::query!(
        "INSERT INTO users (username, password) VALUES (?, ?)",
        packet.username,
        packet.password
    )
    .execute(&server.pool)
    .await
    {
        error!("Failed to create new user due to {}", e);
        send_error(server, addr, "Server error", true).await;
        return;
    }

    let Ok(user) = sqlx::query!("SELECT id FROM users WHERE username = ?", packet.username)
        .fetch_one(&server.pool)
        .await
    else {
        error!("Newly created user cannot be found");
        send_error(server, addr, "Server error", true).await;
        return;
    };

    if let Err(e) = sqlx::query!("INSERT INTO characters (name, position_x, position_y, position_z, owner) VALUES (?, ?, ?, ?, ?)", packet.username, 0.0, 0.0, 0.0, user.id).execute(&server.pool).await {
        error!("Failed to insert new character for {} due to {}", packet.username, e);
        send_error(server, addr, "Server error", true).await;
        return;
    }
}

```


=== components::components.rs

```pretty-rs
// components.rs
use crate::{
    input::Mouse,
    ui::{self, Element, Rectangle, Region, SizeConstraints},
};
use glam::{UVec2, Vec4};

#[derive(Clone, Debug)]
pub struct Container<T: Element> {
    pub child: T,
    pub color: Vec4,
    pub border_radius: u32,
    pub border_color: Vec4,
}

impl<T: Element> Element for Container<T> {
    fn layout(&mut self, constraint: SizeConstraints) -> UVec2 {
        let max = constraint.max - UVec2::new(self.border_radius, self.border_radius);
        let child_size = self.child.layout(SizeConstraints {
            min: constraint.min,
            max,
        });

        child_size + UVec2::new(self.border_radius * 2, self.border_radius * 2)
    }

    fn paint(&mut self, region: Region, scene: &mut Vec<Rectangle>) {
        scene.push(Rectangle {
            color: self.border_color,
            origin: region.origin,
            extent: region.size,
            radius: self.border_radius,
            ..Default::default()
        });
        scene.push(Rectangle {
            color: self.color,
            origin: region.origin + UVec2::new(self.border_radius, self.border_radius),
            extent: region.size - UVec2::new(self.border_radius * 2, self.border_radius * 2),
            ..Default::default()
        });

        self.child.paint(
            Region {
                origin: region.origin + UVec2::new(self.border_radius, self.border_radius),
                size: region.size - UVec2::new(self.border_radius * 2, self.border_radius * 2),
            },
            scene,
        )
    }
}

#[derive(Clone, Debug)]
pub struct Padding<T: Element> {
    pub child: T,
    pub top: u32,
    pub bottom: u32,
    pub left: u32,
    pub right: u32,
}

impl<T: Element> Padding<T> {
    pub fn new_uniform(child: T, padding: u32) -> Self {
        Self {
            child,
            top: padding,
            bottom: padding,
            left: padding,
            right: padding,
        }
    }
}

impl<T: Element> Element for Padding<T> {
    fn layout(&mut self, constraint: SizeConstraints) -> UVec2 {
        let max = constraint.max - UVec2::new(self.left + self.right, self.top + self.bottom);
        let child_size = self.child.layout(SizeConstraints {
            min: constraint.min,
            max,
        });

        child_size + UVec2::new(self.left + self.right, self.top + self.bottom)
    }

    fn paint(&mut self, region: Region, scene: &mut Vec<Rectangle>) {
        self.child.paint(
            Region {
                origin: region.origin + UVec2::new(self.left, self.top),
                size: region.size - UVec2::new(self.left + self.right, self.top + self.bottom),
            },
            scene,
        );
    }
}

#[derive(Debug)]
pub struct PaddingRef<'a, T: Element> {
    pub child: &'a mut T,
    pub top: u32,
    pub bottom: u32,
    pub left: u32,
    pub right: u32,
}

impl<'a, T: Element> PaddingRef<'a, T> {
    pub fn new_uniform(child: &'a mut T, padding: u32) -> Self {
        Self {
            child,
            top: padding,
            bottom: padding,
            left: padding,
            right: padding,
        }
    }
}

impl<T: Element> Element for PaddingRef<'_, T> {
    fn layout(&mut self, constraint: SizeConstraints) -> UVec2 {
        let max = constraint.max - UVec2::new(self.left + self.right, self.top + self.bottom);
        let child_size = self.child.layout(SizeConstraints {
            min: constraint.min,
            max,
        });

        child_size + UVec2::new(self.left + self.right, self.top + self.bottom)
    }

    fn paint(&mut self, region: Region, scene: &mut Vec<Rectangle>) {
        self.child.paint(
            Region {
                origin: region.origin + UVec2::new(self.left, self.top),
                size: region.size - UVec2::new(self.left + self.right, self.top + self.bottom),
            },
            scene,
        );
    }
}

#[derive(Clone, Copy, Debug)]
pub enum VAlign {
    Top,
    Bottom,
    Center,
}

#[derive(Clone, Debug)]
pub struct HPair<L: Element, R: Element> {
    pub left: L,
    pub right: R,
    pub align: VAlign,
    pub separation: u32,
    left_size: UVec2,
    right_size: UVec2,
}

impl<L: Element, R: Element> HPair<L, R> {
    pub fn new(left: L, right: R, align: VAlign, separation: u32) -> Self {
        Self {
            left,
            right,
            align,
            separation,
            left_size: UVec2::ZERO,
            right_size: UVec2::ZERO,
        }
    }

    fn get_top_padding(&self, wanted: u32, actual: u32) -> u32 {
        println!("Wanted: {}, Actual: {}", wanted, actual);
        match self.align {
            VAlign::Top => 0,
            VAlign::Bottom => wanted - actual,
            VAlign::Center => (wanted - actual) / 2,
        }
    }

    fn get_bottom_padding(&self, wanted: u32, actual: u32) -> u32 {
        (wanted - actual) - self.get_top_padding(wanted, actual)
    }
}

impl<L: Element, R: Element> Element for HPair<L, R> {
    fn layout(&mut self, constraint: SizeConstraints) -> UVec2 {
        self.left_size = self.left.layout(constraint.clone());
        self.right_size = self.right.layout(SizeConstraints {
            min: constraint.min,
            max: constraint.max - UVec2::new(self.left_size.x + self.separation, 0),
        });

        UVec2::new(
            self.left_size.x + self.right_size.x + self.separation,
            *[self.left_size.y, self.right_size.y].iter().max().unwrap(),
        )
    }

    fn paint(&mut self, region: Region, scene: &mut Vec<Rectangle>) {
        {
            let top = self.get_top_padding(region.size.y, self.left_size.y);
            let bottom = self.get_bottom_padding(region.size.y, self.left_size.y);
            let mut left = PaddingRef {
                child: &mut self.left,
                top,
                bottom,
                left: 0,
                right: 0,
            };
            left.paint(
                Region {
                    origin: region.origin,
                    size: UVec2::new(self.left_size.x, region.size.y),
                },
                scene,
            );
        }

        {
            let top = self.get_top_padding(region.size.y, self.right_size.y);
            let bottom = self.get_bottom_padding(region.size.y, self.right_size.y);
            let mut right = PaddingRef {
                child: &mut self.right,
                top,
                bottom,
                left: 0,
                right: 0,
            };
            right.paint(
                Region {
                    origin: region.origin + UVec2::new(self.left_size.x + self.separation, 0),
                    size: UVec2::new(self.right_size.x, region.size.y),
                },
                scene,
            );
        }
    }
}

pub const CHAR_HEIGHT: u32 = 5;

static CHARACTER_MAP: [(char, u32); 38] = [
    ('A', 5),
    ('B', 5),
    ('C', 5),
    ('D', 5),
    ('E', 5),
    ('F', 5),
    ('G', 5),
    ('H', 5),
    ('I', 5),
    ('J', 5),
    ('K', 5),
    ('L', 5),
    ('M', 5),
    ('N', 5),
    ('O', 5),
    ('P', 5),
    ('Q', 5),
    ('R', 5),
    ('S', 5),
    ('T', 5),
    ('U', 5),
    ('V', 5),
    ('W', 5),
    ('X', 5),
    ('Y', 5),
    ('Z', 5),
    (' ', 5),
    ('0', 5),
    ('1', 3),
    ('2', 4),
    ('3', 4),
    ('4', 4),
    ('5', 4),
    ('6', 4),
    ('7', 4),
    ('8', 4),
    ('9', 4),
    ('/', 5),
];

#[derive(Clone, Debug)]
pub struct Text {
    pub color: Vec4,
    pub content: String,
}

impl Element for Text {
    fn layout(&mut self, constraint: SizeConstraints) -> UVec2 {
        let width = self
            .content
            .to_uppercase()
            .chars()
            .map(|c| {
                CHARACTER_MAP
                    .iter()
                    .find(|a| a.0 == c)
                    .expect(&format!("Character {} not in font", c))
                    .1
            })
            .fold(0, |acc, w| acc + w + 1);

        UVec2::new(
            width.max(constraint.min.x),
            CHAR_HEIGHT.max(constraint.min.y),
        )
    }

    fn paint(&mut self, region: Region, scene: &mut Vec<Rectangle>) {
        let mut offset = 0;
        for c in self.content.to_uppercase().chars() {
            let (atlas_id, (_, width)) = CHARACTER_MAP
                .iter()
                .enumerate()
                .find(|(_, a)| a.0 == c)
                .expect(&format!("Character {} not in font", c));

            scene.push(Rectangle {
                color: self.color,
                origin: region.origin + UVec2::new(offset, 0),
                extent: UVec2::new(*width, 5),
                atlas_id: atlas_id as i32,
                ..Default::default()
            });

            offset += width + 1;
        }
    }
}

#[derive(Clone, Debug)]
pub enum HAlign {
    Left,
    Right,
    Center,
}

#[derive(Clone, Debug)]
pub struct VList<T: Element> {
    pub children: Vec<T>,
    pub separation: u32,
    pub align: HAlign,
}

// TODO: Alignment
impl<T: Element> Element for VList<T> {
    fn layout(&mut self, constraint: SizeConstraints) -> UVec2 {
        if self.children.len() == 0 {
            return UVec2::new(0, 0);
        }

        let children_sizes = self
            .children
            .iter_mut()
            .map(|child| child.layout(constraint.clone()))
            .collect::<Vec<UVec2>>();
        let width = children_sizes.iter().map(|size| size.x).max().unwrap();
        let height = children_sizes.first().unwrap().y * self.children.len() as u32
            + self.separation * (self.children.len() as u32 - 1);

        UVec2 {
            x: width,
            y: height,
        }
    }

    fn paint(&mut self, region: Region, scene: &mut Vec<Rectangle>) {
        if self.children.len() == 0 {
            return;
        }

        let height_per_child = (region.size.y + self.separation
            - (self.children.len() as u32 * self.separation))
            / (self.children.len() as u32);

        for (i, child) in self.children.iter_mut().enumerate() {
            child.paint(
                Region {
                    origin: region.origin
                        + UVec2::new(0, (height_per_child + self.separation) * i as u32),
                    size: UVec2::new(region.size.x, height_per_child),
                },
                scene,
            );
        }
    }
}

#[derive(Clone, Debug)]
pub struct VPair<T: Element, B: Element> {
    pub top: T,
    pub bottom: B,
    pub align: HAlign,
    pub separation: u32,
    top_size: UVec2,
    bottom_size: UVec2,
}

impl<T: Element, B: Element> VPair<T, B> {
    pub fn new(top: T, bottom: B, align: HAlign, separation: u32) -> Self {
        Self {
            top,
            bottom,
            align,
            separation,
            top_size: UVec2::ZERO,
            bottom_size: UVec2::ZERO,
        }
    }

    fn get_left_padding(&self, wanted: u32, actual: u32) -> u32 {
        println!("Wanted: {}, Actual: {}", wanted, actual);
        match self.align {
            HAlign::Left => 0,
            HAlign::Right => wanted - actual,
            HAlign::Center => (wanted - actual) / 2,
        }
    }

    fn get_right_padding(&self, wanted: u32, actual: u32) -> u32 {
        (wanted - actual) - self.get_left_padding(wanted, actual)
    }
}

impl<T: Element, B: Element> Element for VPair<T, B> {
    fn layout(&mut self, constraint: SizeConstraints) -> UVec2 {
        self.top_size = self.top.layout(constraint.clone());
        self.bottom_size = self.bottom.layout(SizeConstraints {
            min: constraint.min,
            max: constraint.max - UVec2::new(0, self.top_size.y + self.separation),
        });

        UVec2::new(
            *[self.top_size.x, self.bottom_size.x].iter().max().unwrap(),
            self.top_size.y + self.bottom_size.y + self.separation,
        )
    }

    fn paint(&mut self, region: Region, scene: &mut Vec<Rectangle>) {
        {
            let left = self.get_left_padding(region.size.x, self.top_size.x);
            let right = self.get_right_padding(region.size.x, self.top_size.x);
            let mut top = PaddingRef {
                child: &mut self.top,
                left,
                right,
                top: 0,
                bottom: 0,
            };
            top.paint(
                Region {
                    origin: region.origin,
                    size: UVec2::new(region.size.x, self.top_size.y),
                },
                scene,
            );
        }

        {
            let left = self.get_left_padding(region.size.x, self.bottom_size.x);
            let right = self.get_right_padding(region.size.x, self.bottom_size.x);
            let mut bottom = PaddingRef {
                child: &mut self.bottom,
                left,
                right,
                top: 0,
                bottom: 0,
            };
            bottom.paint(
                Region {
                    origin: region.origin + UVec2::new(0, self.top_size.y + self.separation),
                    size: UVec2::new(region.size.x, self.bottom_size.y),
                },
                scene,
            );
        }
    }
}

pub struct Button<'a, H: Handler> {
    component: Container<Padding<Text>>,
    mouse: &'a Mouse,
    on_click: H,
}

pub trait Handler {
    fn handle(&mut self);
}

impl<'a, H: Handler> Button<'a, H> {
    pub fn new(mouse: &'a Mouse, text: &str, on_click: H) -> Self {
        Self {
            component: Container {
                child: Padding::new_uniform(
                    Text {
                        color: ui::color::get_highlight(),
                        content: text.to_owned(),
                    },
                    3,
                ),
                color: ui::color::get_background(),
                border_color: ui::color::get_highlight(),
                border_radius: 1,
            },
            mouse,
            on_click,
        }
    }
}

impl<H: Handler> Element for Button<'_, H> {
    fn layout(&mut self, constraint: ui::SizeConstraints) -> glam::UVec2 {
        self.component.layout(constraint)
    }

    fn paint(&mut self, region: ui::Region, scene: &mut Vec<ui::Rectangle>) {
        if ui::input::hovering(self.mouse, &region) {
            self.component.color = Vec4::ONE;
        } else {
            self.component.color = ui::color::get_background();
        }

        if ui::input::clicked(self.mouse, &region, winit::event::MouseButton::Left) {
            self.on_click.handle()
        }

        self.component.paint(region, scene)
    }
}

```


=== aetheria::main.rs

```pretty-rs
// main.rs
#![feature(let_chains)]
#![feature(trivial_bounds)]
#![feature(associated_type_defaults)]
#![warn(clippy::pedantic)]
#![warn(clippy::nursery)]

extern crate core;

mod camera;
mod components;
mod data;
mod entities;
mod input;
mod macros;
mod renderer;
mod scenes;
mod socket;
mod systems;
mod time;
mod ui;

use anyhow::Result;
use ash::vk;
use assets::{ModelRegistry, ShaderRegistry, TextureRegistry, Transform};
use bytemuck::cast_slice;
use camera::Camera;
use common::{
    item::{Item, ItemStack},
    net, Observable, Observer,
};
use glam::{IVec2, Quat, UVec2, Vec2, Vec3, Vec4};
use input::{Keyboard, Mouse};
use num_traits::FromPrimitive;
use std::{
    collections::HashMap,
    f32::consts::PI,
    io,
    net::{IpAddr, SocketAddr, UdpSocket},
    ops::DerefMut,
    sync::{Arc, Mutex},
    time::Instant,
};
use time::Time;
use tracing::info;
use vulkan::Context;
use winit::{
    event::{MouseButton, VirtualKeyCode},
    event_loop::ControlFlow,
};

use crate::{
    components::{craft, recipe_selector},
    data::{inventory::Inventory, Data},
    entities::{Player, Tree},
    renderer::{Renderer, RENDER_HEIGHT, RENDER_WIDTH},
    scenes::RootScene,
    socket::Socket,
    systems::{interact, render, Systems},
    ui::{Element, Rectangle, Region, SizeConstraints, UIPass},
};

use dialog::DialogBox;

struct Indices(Vec<u32>);
impl From<Indices> for Vec<u8> {
    fn from(indices: Indices) -> Self {
        cast_slice::<u32, u8>(&indices.0).to_vec()
    }
}

fn create_window() -> (winit::event_loop::EventLoop<()>, winit::window::Window) {
    let event_loop = winit::event_loop::EventLoop::new();
    let window = winit::window::WindowBuilder::new()
        .build(&event_loop)
        .unwrap();
    (event_loop, window)
}

const CAMERA_SENSITIVITY: f32 = 250.0;

fn main() {
    tracing_subscriber::fmt::init();

    let mut ip = dialog::Input::new("Enter Server IP:")
        .title("IP")
        .show()
        .expect("Failed to show IP dialog box")
        .unwrap_or("".to_owned());

    if ip.trim().is_empty() {
        ip = "127.0.0.1".to_owned();
    }

    let remote = SocketAddr::new(IpAddr::V4(ip.trim().parse().unwrap()), 8000);
    let socket: Arc<Socket> = Arc::new(UdpSocket::bind("[::]:0").unwrap().into());
    socket.connect(remote).unwrap();
    socket.set_nonblocking(true).unwrap();

    let username = dialog::Input::new("Enter username:")
        .title("Username")
        .show()
        .expect("Failed to show username dialog box");

    if username.is_none() || username.as_ref().unwrap().trim().is_empty() {
        dialog::Message::new("Username cannot be empty")
            .title("Username error")
            .show()
            .expect("Failed to show error dialog box");

        return;
    }
    let username = username.unwrap();

    let password = dialog::Password::new("Enter password:")
        .title("Password")
        .show()
        .expect("Failed to show password dialog box");

    if password.is_none() || password.as_ref().unwrap().trim().is_empty() {
        dialog::Message::new("Password cannot be empty")
            .title("Password error")
            .show()
            .expect("Failed to show error dialog box");

        return;
    }
    let password = password.unwrap();

    match dialog::Question::new("Do you have an existing account")
        .title("Existing account")
        .show()
        .unwrap()
    {
        dialog::Choice::Yes => {
            let login = net::server::Packet::Login(net::server::Login {
                username: username.trim().to_owned(),
                password: password.trim().to_owned(),
            });

            socket.send(&login).unwrap();
        }
        dialog::Choice::No => {
            let signup = net::server::Packet::Signup(net::server::Signup {
                username: username.trim().to_owned(),
                password: password.trim().to_owned(),
            });

            socket.send(&signup).unwrap();
        }
        dialog::Choice::Cancel => return,
    };

    let (event_loop, window) = create_window();
    let window = Arc::new(window);
    let ctx = Context::new(&window);

    let mut model_registry = ModelRegistry::new();
    let mut shader_registry = ShaderRegistry::new();
    let mut texture_registry = TextureRegistry::new();

    let mut renderer = Renderer::new(ctx, window.clone()).unwrap();
    let mut camera = Camera::new(480.0, 270.0, &renderer).unwrap();
    let mut time = Time::new(&renderer).unwrap();
    let render_system = Arc::new(Mutex::new(
        render::System::new(&renderer, &mut shader_registry, &camera, &time).unwrap(),
    ));
    let interact_system = Arc::new(Mutex::new(interact::System::new()));

    let mut data = Data {
        inventory: Inventory::new(socket.clone()),
        current_recipe: None,
        recipe_selections: None,
    };

    let ui_pass = Arc::new(Mutex::new(
        UIPass::new(
            &mut renderer,
            &mut shader_registry,
            &mut texture_registry,
            render_system.lock().unwrap().get_texture(),
        )
        .unwrap(),
    ));
    renderer.add_pass(render_system.clone());
    renderer.add_pass(ui_pass.clone());
    renderer.set_output_image(
        ui_pass.lock().unwrap().get_texture().image.clone(),
        vk::ImageLayout::GENERAL,
    );
    let mut keyboard = Keyboard::new();
    let mut mouse = Mouse::new();

    let mut root = RootScene::new(
        &mut renderer,
        &mut Systems {
            render: &mut render_system.lock().unwrap(),
            interact: &mut interact_system.lock().unwrap(),
        },
        &mut model_registry,
    )
    .expect("Failed to load scene");

    interact_system
        .lock()
        .unwrap()
        .set_player(root.player.clone());

    let mut players: HashMap<String, Arc<Mutex<Player>>> = HashMap::new();
    let mut last_heartbeat: Instant = Instant::now();

    let mut inventory_open = false;

    event_loop.run(move |event, _, control_flow| {
        if let ControlFlow::ExitWithCode(_) = *control_flow {
            return;
        }

        control_flow.set_poll();

        keyboard.on_event(&event);
        mouse.on_event(&event);

        let mut buf = [0; 4096];
        match socket.recv(&mut buf) {
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {}
            Err(e) => panic!("{e}"),
            Ok(_) => {
                let packet: net::client::Packet = postcard::from_bytes(&buf).unwrap();

                match packet {
                    net::client::Packet::SpawnPlayer(packet) => {
                        info!("Spawning player");
                        players.insert(
                            packet.username,
                            Player::new(
                                &mut renderer,
                                &mut Systems {
                                    render: &mut render_system.lock().unwrap(),
                                    interact: &mut interact_system.lock().unwrap(),
                                },
                                &mut model_registry,
                                Transform {
                                    translation: packet.position,
                                    rotation: Quat::IDENTITY,
                                    scale: Vec3::ONE,
                                },
                            )
                            .unwrap(),
                        );
                    }
                    net::client::Packet::Move(packet) => {
                        info!("Moving peer player");
                        players
                            .get_mut(&packet.username)
                            .expect("Peer not found")
                            .lock()
                            .unwrap()
                            .player
                            .transform
                            .translation = packet.position;
                    }
                    net::client::Packet::DespawnPlayer(packet) => {
                        info!("Deleting peer player");
                        players.remove(&packet.username);
                    }
                    net::client::Packet::NotifyDisconnection(packet) => {
                        info!("Disconnecting due to {}", packet.reason);
                        control_flow.set_exit();
                        return;
                    }
                    net::client::Packet::ModifyInventory(packet) => {
                        info!("Setting {:?} to {}", packet.stack.item, packet.stack.amount);
                        data.inventory.set(packet.stack);
                    }
                    net::client::Packet::DisplayError(packet) => {
                        dialog::Message::new(packet.message)
                            .title("Error")
                            .show()
                            .unwrap();
                        if packet.fatal {
                            control_flow.set_exit();
                            return;
                        }
                    }
                }
            }
        };

        if last_heartbeat.elapsed().as_secs_f32() > 10.0 {
            heartbeat(&socket).unwrap();
            last_heartbeat = Instant::now();
        }

        match event {
            winit::event::Event::WindowEvent { event, .. } => match event {
                winit::event::WindowEvent::Resized(size) => {
                    renderer.recreate_swapchain().unwrap();
                    camera.width = size.width as f32;
                    camera.height = size.height as f32;
                }
                winit::event::WindowEvent::CloseRequested => {
                    disconnect(&socket).unwrap();
                    control_flow.set_exit()
                }
                _ => (),
            },
            winit::event::Event::MainEventsCleared => {
                if keyboard.is_key_down(VirtualKeyCode::Escape) {
                    disconnect(&socket).unwrap();
                    control_flow.set_exit()
                }
                if mouse.is_button_down(MouseButton::Right) {
                    camera.theta -= mouse.delta.x / CAMERA_SENSITIVITY
                }
                if keyboard.is_key_down(VirtualKeyCode::Left) {
                    let mut sun = root.sun.lock().unwrap();
                    let theta = sun.get_theta() + PI / 60.0;
                    sun.update_theta(theta);
                }
                if keyboard.is_key_down(VirtualKeyCode::Right) {
                    let mut sun = root.sun.lock().unwrap();
                    let theta = sun.get_theta() - PI / 60.0;
                    sun.update_theta(theta);
                }

                if keyboard.is_key_pressed(VirtualKeyCode::I) {
                    inventory_open = !inventory_open;
                }

                renderer.wait_for_frame();
                render_system
                    .lock()
                    .unwrap()
                    .set_geometry(&data, &renderer, &model_registry);

                let mut scene = Vec::new();

                interact_system
                    .lock()
                    .unwrap()
                    .frame_finished(&camera, &keyboard, &mut scene, &mut data);

                if let Some(mut component) = craft::Component::new(&mut data, &mouse) {
                    let size = component.layout(SizeConstraints {
                        min: UVec2::new(0, 0),
                        max: UVec2::new(480, 270),
                    });
                    component.paint(
                        Region {
                            origin: UVec2::new(0, 0),
                            size,
                        },
                        &mut scene,
                    )
                }

                if let Some(mut component) = recipe_selector::Component::new(&mut data, &mouse) {
                    let size = component.layout(SizeConstraints {
                        min: UVec2::new(0, 0),
                        max: UVec2::new(480, 270),
                    });
                    component.paint(
                        Region {
                            origin: UVec2::new(0, 0),
                            size,
                        },
                        &mut scene,
                    )
                }

                if inventory_open {
                    let mut inventory_window =
                        components::inventory::Component::new(&data.inventory);
                    let size = inventory_window.layout(SizeConstraints {
                        min: UVec2::new(0, 0),
                        max: UVec2::new(RENDER_WIDTH, RENDER_HEIGHT),
                    });
                    inventory_window.paint(
                        Region {
                            origin: UVec2::new(480 - (size.x + 2), 270 - (size.y + 2)),
                            size,
                        },
                        &mut scene,
                    );
                }
                ui_pass
                    .lock()
                    .unwrap()
                    .set_geometry(&renderer, &scene)
                    .expect("Failed to set UI geometry");

                renderer.render();
                let viewport = Vec2::new(
                    window.inner_size().width as f32,
                    window.inner_size().height as f32,
                );

                root.frame_finished(&keyboard, &mouse, &camera, &time, viewport, &socket);
                time.frame_finished();
                keyboard.frame_finished();
                camera.frame_finished();
                mouse.frame_finished();
                camera.target = root.player.lock().unwrap().player.transform.translation;

                println!("{}", mouse.position);
            }
            _ => (),
        };

        if let ControlFlow::ExitWithCode(_) = *control_flow {
            println!("Waiting for GPU to finish");
            unsafe { renderer.device.device_wait_idle().unwrap() };
        }
    });
}

fn heartbeat(socket: &Socket) -> Result<()> {
    let packet = net::server::Packet::Heartbeat;
    socket.send(&packet)?;
    Ok(())
}

fn disconnect(socket: &Socket) -> Result<()> {
    let packet = net::server::Packet::Disconnect;
    socket.send(&packet)?;
    Ok(())
}

```


=== vulkan::command.rs

```pretty-rs
// command.rs
use super::{compute, graphics, Device, Image, Renderpass, Set};
use ash::vk;
use std::{ops::Deref, result::Result, sync::Arc};

#[derive(Clone, Copy, Debug, Default)]
pub struct DrawOptions {
    pub vertex_count: u32,
    pub instance_count: u32,
    pub first_vertex: i32,
    pub first_instance: u32,
}

#[derive(Clone, Debug)]
pub struct Buffer {
    pub(crate) buffer: vk::CommandBuffer,
}

enum Pipeline {
    Graphics(graphics::Pipeline),
    Compute(compute::Pipeline),
}

impl Pipeline {
    pub fn get_layout(&self) -> vk::PipelineLayout {
        match self {
            Pipeline::Graphics(graphics) => graphics.layout,
            Pipeline::Compute(compute) => compute.layout,
        }
    }

    pub fn get_bind_point(&self) -> vk::PipelineBindPoint {
        match self {
            Pipeline::Compute(_) => vk::PipelineBindPoint::COMPUTE,
            Pipeline::Graphics(_) => vk::PipelineBindPoint::GRAPHICS,
        }
    }
}

pub struct BufferBuilder {
    buffer: Buffer,
    device: Arc<Device>,
    pipeline: Option<Pipeline>,
}

#[derive(Clone, Debug)]
pub struct TransitionLayoutOptions {
    pub old: vk::ImageLayout,
    pub new: vk::ImageLayout,
    pub source_access: vk::AccessFlags,
    pub destination_access: vk::AccessFlags,
    pub source_stage: vk::PipelineStageFlags,
    pub destination_stage: vk::PipelineStageFlags,
}

impl BufferBuilder {
    pub fn begin(self) -> Result<Self, vk::Result> {
        let begin_info = vk::CommandBufferBeginInfo::builder();
        unsafe { self.device.begin_command_buffer(**self, &begin_info)? };
        Ok(self)
    }

    pub fn begin_renderpass(
        self,
        renderpass: &Renderpass,
        framebuffer: vk::Framebuffer,
        extent: vk::Extent2D,
    ) -> Self {
        let render_area = vk::Rect2D::builder()
            .offset(vk::Offset2D::default())
            .extent(extent);

        let color_clear_value = vk::ClearValue {
            color: vk::ClearColorValue {
                float32: [0.0, 0.0, 0.0, 1.0],
            },
        };

        let depth_clear_value = vk::ClearValue {
            depth_stencil: vk::ClearDepthStencilValue {
                depth: 1.0,
                stencil: 0,
            },
        };

        let clear_values = &[color_clear_value, depth_clear_value];
        let begin_info = vk::RenderPassBeginInfo::builder()
            .render_pass(**renderpass)
            .framebuffer(framebuffer)
            .render_area(*render_area)
            .clear_values(clear_values);

        unsafe {
            self.device
                .cmd_begin_render_pass(**self, &begin_info, vk::SubpassContents::INLINE)
        };

        self
    }

    pub fn bind_graphics_pipeline(mut self, pipeline: graphics::Pipeline) -> Self {
        unsafe {
            self.device
                .cmd_bind_pipeline(**self, vk::PipelineBindPoint::GRAPHICS, *pipeline)
        };

        self.pipeline = Some(Pipeline::Graphics(pipeline));

        self
    }

    pub fn bind_compute_pipeline(mut self, pipeline: compute::Pipeline) -> Self {
        unsafe {
            self.device
                .cmd_bind_pipeline(**self, vk::PipelineBindPoint::COMPUTE, *pipeline)
        };

        self.pipeline = Some(Pipeline::Compute(pipeline));

        self
    }

    pub fn bind_descriptor_set(self, binding: u32, descriptor_set: &Set) -> Self {
        let descriptor_sets = &[**descriptor_set];
        unsafe {
            self.device.cmd_bind_descriptor_sets(
                **self,
                self.pipeline
                    .as_ref()
                    .expect("Binding descriptor set without pipeline bound")
                    .get_bind_point(),
                self.pipeline.as_ref().unwrap().get_layout(),
                binding,
                descriptor_sets,
                &[],
            );
        }

        self
    }

    pub fn bind_index_buffer(self, index_buffer: &super::Buffer) -> Self {
        unsafe {
            self.device
                .cmd_bind_index_buffer(**self, **index_buffer, 0, vk::IndexType::UINT32)
        };

        self
    }

    pub fn bind_vertex_buffer(self, vertex_buffer: &super::Buffer) -> Self {
        unsafe {
            self.device
                .cmd_bind_vertex_buffers(**self, 0, &[**vertex_buffer], &[0])
        };

        self
    }

    pub fn next_subpass(self) -> Self {
        unsafe {
            self.device
                .cmd_next_subpass(**self, vk::SubpassContents::INLINE)
        };

        self
    }

    pub fn draw(self, options: DrawOptions) -> Self {
        unsafe {
            self.device.cmd_draw_indexed(
                **self,
                options.vertex_count,
                options.instance_count,
                0,
                options.first_vertex,
                options.first_instance,
            );
        };

        self
    }

    pub fn dispatch(self, x: u32, y: u32, z: u32) -> Self {
        unsafe {
            self.device.cmd_dispatch(**self, x, y, z);
        }

        self
    }

    pub fn blit_image(
        self,
        from: &Image,
        to: &Image,
        from_layout: vk::ImageLayout,
        to_layout: vk::ImageLayout,
        aspect: vk::ImageAspectFlags,
        filter: vk::Filter,
    ) -> Self {
        unsafe {
            let subresource = vk::ImageSubresourceLayers::builder()
                .aspect_mask(aspect)
                .mip_level(0)
                .base_array_layer(0)
                .layer_count(1);
            let copy_info = vk::ImageBlit::builder()
                .src_subresource(*subresource)
                .src_offsets([
                    vk::Offset3D::default(),
                    vk::Offset3D {
                        x: from.width as i32,
                        y: from.height as i32,
                        z: 1,
                    },
                ])
                .dst_subresource(*subresource)
                .dst_offsets([
                    vk::Offset3D::default(),
                    vk::Offset3D {
                        x: to.width as i32,
                        y: to.height as i32,
                        z: 1,
                    },
                ]);
            self.device.cmd_blit_image(
                **self,
                from.image,
                from_layout,
                to.image,
                to_layout,
                &[*copy_info],
                filter,
            );
        }

        self
    }

    pub fn end_renderpass(self) -> Self {
        unsafe { self.device.cmd_end_render_pass(**self) };

        self
    }

    pub fn copy_buffer_to_image(self, buffer: &super::Buffer, image: &Image) -> Self {
        let region = vk::BufferImageCopy::builder()
            .buffer_offset(0)
            .buffer_row_length(0)
            .buffer_image_height(0)
            .image_subresource(vk::ImageSubresourceLayers {
                aspect_mask: vk::ImageAspectFlags::COLOR,
                mip_level: 0,
                base_array_layer: 0,
                layer_count: 1,
            })
            .image_offset(vk::Offset3D { x: 0, y: 0, z: 0 })
            .image_extent(vk::Extent3D {
                width: image.width,
                height: image.height,
                depth: 1,
            });

        let regions = &[*region];
        unsafe {
            self.device.cmd_copy_buffer_to_image(
                **self,
                **buffer,
                **image,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                regions,
            )
        };

        self
    }

    pub fn transition_image_layout(self, image: &Image, options: &TransitionLayoutOptions) -> Self {
        let barrier = vk::ImageMemoryBarrier::builder()
            .src_access_mask(options.source_access)
            .dst_access_mask(options.destination_access)
            .old_layout(options.old)
            .new_layout(options.new)
            .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .image(**image)
            .subresource_range(vk::ImageSubresourceRange {
                aspect_mask: vk::ImageAspectFlags::COLOR,
                base_mip_level: 0,
                level_count: 1,
                base_array_layer: 0,
                layer_count: 1,
            });

        let image_memory_barriers = &[*barrier];
        unsafe {
            self.device.cmd_pipeline_barrier(
                **self,
                options.source_stage,
                options.destination_stage,
                vk::DependencyFlags::empty(),
                &[],
                &[],
                image_memory_barriers,
            )
        };

        self
    }

    pub fn clear(self, image: Arc<Image>, color: [f32; 4], layout: vk::ImageLayout) -> Self {
        unsafe {
            let clear_value = vk::ClearColorValue { float32: color };
            let subresource_range = vk::ImageSubresourceRange::builder()
                .aspect_mask(vk::ImageAspectFlags::COLOR)
                .base_mip_level(0)
                .level_count(1)
                .base_array_layer(0)
                .layer_count(1);
            let subresource_ranges = &[*subresource_range];
            self.device.cmd_clear_color_image(
                **self,
                image.image,
                layout,
                &clear_value,
                subresource_ranges,
            );
        }

        self
    }

    pub fn record<F: Fn(Self) -> Self>(self, predicate: F) -> Self {
        predicate(self)
    }

    pub fn end(self) -> Result<Buffer, vk::Result> {
        unsafe { self.device.end_command_buffer(**self)? };

        Ok(self.buffer)
    }

    pub fn submit(self) -> Result<(), vk::Result> {
        unsafe { self.device.end_command_buffer(**self)? };

        let command_buffers = &[**self];
        let submit_info = vk::SubmitInfo::builder().command_buffers(command_buffers);

        let submits = &[*submit_info];
        unsafe {
            self.device
                .queue_submit(*self.device.queues.graphics, submits, vk::Fence::null())?
        };
        unsafe { self.device.queue_wait_idle(*self.device.queues.graphics)? };

        Ok(())
    }
}

impl Deref for BufferBuilder {
    type Target = Buffer;

    fn deref(&self) -> &Self::Target {
        &self.buffer
    }
}

impl Deref for Buffer {
    type Target = vk::CommandBuffer;

    fn deref(&self) -> &Self::Target {
        &self.buffer
    }
}

pub struct Pool {
    pub(crate) pool: vk::CommandPool,
    buffers: Vec<Buffer>,
    device: Arc<Device>,
}

impl Pool {
    pub fn new(device: Arc<Device>) -> Result<Self, vk::Result> {
        let create_info =
            vk::CommandPoolCreateInfo::builder().queue_family_index(device.queues.graphics.index);

        let pool = unsafe { device.create_command_pool(&create_info, None)? };

        Ok(Self {
            pool,
            buffers: Vec::new(),
            device,
        })
    }

    pub fn allocate(&mut self) -> Result<BufferBuilder, vk::Result> {
        let allocate_info = vk::CommandBufferAllocateInfo::builder()
            .command_pool(self.pool)
            .level(vk::CommandBufferLevel::PRIMARY)
            .command_buffer_count(1);

        let buffer = unsafe { self.device.allocate_command_buffers(&allocate_info)?[0] };
        let buffer = Buffer { buffer };
        self.buffers.push(buffer.clone());

        let builder = BufferBuilder {
            buffer,
            device: self.device.clone(),
            pipeline: None,
        };

        Ok(builder)
    }

    pub fn clear(&mut self) {
        if self.buffers.is_empty() {
            return;
        }

        unsafe {
            self.device.free_command_buffers(
                **self,
                &self
                    .buffers
                    .iter()
                    .map(|buffer| **buffer)
                    .collect::<Vec<vk::CommandBuffer>>(),
            );
        }

        self.buffers = Vec::new();
    }
}

impl Deref for Pool {
    type Target = vk::CommandPool;

    fn deref(&self) -> &Self::Target {
        &self.pool
    }
}

```


=== gltf::lib.rs

```pretty-rs
// lib.rs
#![feature(exact_size_is_empty)]

use std::{
    collections::HashMap,
    fmt::Debug,
    io::{Cursor, Read},
};

use serde::{Deserialize, Serialize};
use serde_repr::{Deserialize_repr, Serialize_repr};

#[derive(Deserialize_repr, Serialize_repr, Debug)]
#[repr(u16)]
pub enum ComponentType {
    I8 = 5120,
    U8 = 5121,
    I16 = 5122,
    U16 = 5123,
    U32 = 5125,
    F32 = 5126,
}

impl ComponentType {
    pub fn size_of(&self) -> usize {
        match self {
            Self::I8 | Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::U32 | Self::F32 => 4,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Accessor {
    #[serde(rename = "bufferView")]
    pub buffer_view: usize,
    #[serde(default)]
    #[serde(rename = "byteOffset")]
    pub byte_offset: usize,
    #[serde(rename = "componentType")]
    pub component_type: ComponentType,
    #[serde(default)]
    pub normalized: bool,
    pub count: usize,
    #[serde(rename = "type")]
    pub element_type: String,
    #[serde(default)]
    pub max: Option<Vec<f64>>,
    #[serde(default)]
    pub min: Option<Vec<f64>>,
}

impl Accessor {
    pub fn get_data(&self, glb: &Glb) -> Vec<u8> {
        let buffer_view = glb.gltf.buffer_views.get(self.buffer_view).unwrap();
        let buffer = glb.gltf.buffers.get(buffer_view.buffer).unwrap();

        let offset = self.byte_offset + buffer_view.byte_offset;

        let element_size = match self.element_type.as_str() {
            "SCALAR" => 1,
            "VEC2" => 2,
            "VEC3" => 3,
            "VEC4" | "MAT2" => 4,
            "MAT3" => 9,
            "MAT4" => 16,
            _ => panic!("Invalid element type"),
        };
        let size = self.component_type.size_of() * element_size * self.count;

        glb.buffer[offset..(offset + size)].to_vec()
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Animation {}

#[derive(Serialize, Deserialize, Debug)]
pub struct Asset {
    #[serde(default)]
    pub copyright: Option<String>,
    #[serde(default)]
    pub generator: Option<String>,
    pub version: String,
    #[serde(default)]
    #[serde(rename = "minVersion")]
    pub min_version: Option<String>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Buffer {
    #[serde(default)]
    pub uri: String,
    #[serde(rename = "byteLength")]
    pub byte_length: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct BufferView {
    pub buffer: usize,
    #[serde(default)]
    #[serde(rename = "byteOffset")]
    pub byte_offset: usize,
    #[serde(rename = "byteLength")]
    pub byte_length: usize,
    #[serde(default)]
    #[serde(rename = "byteStride")]
    pub byte_stride: usize,
    #[serde(default)]
    pub target: Option<u32>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Camera {}

#[derive(Serialize, Deserialize, Debug)]
pub struct Image {
    #[serde(default)]
    pub uri: Option<String>,
    #[serde(default)]
    #[serde(rename = "mimeType")]
    pub mime_type: Option<String>,
    #[serde(default)]
    #[serde(rename = "bufferView")]
    pub buffer_view: Option<usize>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct TextureInfo {
    pub index: usize,
    #[serde(default)]
    #[serde(rename = "texCoord")]
    pub tex_coord: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MaterialPBR {
    #[serde(default)]
    #[serde(rename = "baseColorFactor")]
    pub base_color_factor: Option<[f32; 4]>,
    #[serde(default)]
    #[serde(rename = "baseColorTexture")]
    pub base_color_texture: Option<TextureInfo>,
    #[serde(default)]
    #[serde(rename = "metallicFactor")]
    pub metallic_factor: Option<f32>,
    #[serde(default)]
    #[serde(rename = "roughnessFactor")]
    pub roughness_factor: Option<f32>,
    #[serde(default)]
    #[serde(rename = "metallicRoughnessTexture")]
    pub metallic_roughness_texture: Option<TextureInfo>,
}

impl Default for MaterialPBR {
    fn default() -> Self {
        Self {
            base_color_factor: Some([1.0, 1.0, 1.0, 1.0]),
            base_color_texture: None,
            metallic_factor: Some(1.0),
            roughness_factor: Some(1.0),
            metallic_roughness_texture: None,
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MaterialNormalTexture {
    pub index: usize,
    #[serde(default)]
    #[serde(rename = "texCoord")]
    pub tex_coord: usize,
    #[serde(default)]
    pub scale: Option<f32>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct MaterialOcclusionTexture {
    pub index: usize,
    #[serde(default)]
    #[serde(rename = "texCoord")]
    pub tex_coord: usize,
    #[serde(default)]
    pub strength: Option<f32>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Material {
    #[serde(default)]
    #[serde(rename = "pbrMetallicRoughness")]
    pub pbr: MaterialPBR,
    #[serde(default)]
    #[serde(rename = "normalTexture")]
    pub normal_texture: Option<MaterialNormalTexture>,
    #[serde(default)]
    #[serde(rename = "occlusionTexture")]
    pub occlusion_texture: Option<MaterialOcclusionTexture>,
    #[serde(default)]
    #[serde(rename = "emissiveTexture")]
    pub emissive_texture: Option<TextureInfo>,
    #[serde(default)]
    #[serde(rename = "emissiveFavtor")]
    pub emissive_factor: Option<[f32; 3]>,
    #[serde(default)]
    #[serde(rename = "alphaMode")]
    pub alpha_mode: Option<String>,
    #[serde(default)]
    #[serde(rename = "alphaCutoff")]
    pub alpha_cutoff: Option<f32>,
    #[serde(default)]
    #[serde(rename = "doubleSided")]
    pub double_sided: bool,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MeshPrimitive {
    pub attributes: HashMap<String, usize>,
    #[serde(default)]
    pub indices: Option<usize>,
    #[serde(default)]
    pub material: Option<usize>,
}

impl MeshPrimitive {
    pub fn get_attribute_data(&self, glb: &Glb, attribute: &str) -> Option<Vec<u8>> {
        glb.gltf
            .accessors
            .get(*self.attributes.get(attribute)?)
            .map(|accessor| accessor.get_data(glb))
    }

    pub fn get_indices_data(&self, glb: &Glb) -> Option<Vec<u32>> {
        glb.gltf.accessors.get(self.indices?).map(|accessor| {
            let data = accessor.get_data(glb);
            match accessor.component_type {
                ComponentType::U16 => bytemuck::cast_slice::<u8, u16>(&data)
                    .iter()
                    .copied()
                    .map(|short| short as u32)
                    .collect(),
                ComponentType::U32 => bytemuck::cast_slice::<u8, u32>(&data).to_vec(),
                _ => panic!("Invalid index component type"),
            }
        })
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Mesh {
    pub primitives: Vec<MeshPrimitive>,
    #[serde(default)]
    pub weights: Option<Vec<f64>>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Node {
    #[serde(default)]
    pub camera: Option<usize>,
    #[serde(default)]
    pub children: Vec<usize>,
    #[serde(default)]
    pub skin: Option<usize>,
    #[serde(default)]
    pub matrix: Option<[f32; 16]>,
    #[serde(default)]
    pub mesh: Option<usize>,
    #[serde(default)]
    pub rotation: Option<[f32; 4]>,
    #[serde(default)]
    pub scale: Option<[f32; 3]>,
    #[serde(default)]
    pub translation: Option<[f32; 3]>,
    #[serde(default)]
    pub weights: Option<Vec<f64>>,
}

#[derive(Deserialize_repr, Serialize_repr, Debug)]
#[repr(u16)]
pub enum Filter {
    Nearest = 9728,
    Linear = 9729,
    NearestMipmapNearest = 9984,
    LinearMipmapNearest = 9985,
    NearestMipmapLinear = 9986,
    LinearMipmapLinear = 9987,
}

impl Default for Filter {
    fn default() -> Self {
        Self::Linear
    }
}

#[derive(Deserialize_repr, Serialize_repr, Debug)]
#[repr(u16)]
pub enum AddressMode {
    ClampToEdge = 33071,
    MirroredRepeat = 33648,
    Repeat = 10497,
}

impl Default for AddressMode {
    fn default() -> Self {
        Self::Repeat
    }
}

#[derive(Serialize, Deserialize, Default, Debug)]
pub struct Sampler {
    #[serde(default)]
    #[serde(rename = "magFilter")]
    pub mag_filter: Filter,
    #[serde(default)]
    #[serde(rename = "minFilter")]
    pub min_filter: Filter,
    #[serde(default)]
    #[serde(rename = "wrapS")]
    pub wrap_u: AddressMode,
    #[serde(default)]
    #[serde(rename = "wrapT")]
    pub wrap_v: AddressMode,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Scene {
    #[serde(default)]
    pub nodes: Vec<usize>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Skin {}

#[derive(Serialize, Deserialize, Debug)]
pub struct Texture {
    #[serde(default)]
    pub sampler: Option<usize>,
    pub source: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Gltf {
    #[serde(default)]
    #[serde(rename = "extensionsUsed")]
    pub extensions_used: Vec<String>,
    #[serde(default)]
    #[serde(rename = "extensionsRequired")]
    pub extensions_required: Vec<String>,

    #[serde(default)]
    pub accessors: Vec<Accessor>,
    #[serde(default)]
    pub animations: Vec<Animation>,
    pub asset: Asset,
    #[serde(default)]
    pub buffers: Vec<Buffer>,
    #[serde(default)]
    #[serde(rename = "bufferViews")]
    pub buffer_views: Vec<BufferView>,
    #[serde(default)]
    pub cameras: Vec<Camera>,
    #[serde(default)]
    pub images: Vec<Image>,
    #[serde(default)]
    pub materials: Vec<Material>,
    #[serde(default)]
    pub meshes: Vec<Mesh>,
    #[serde(default)]
    pub nodes: Vec<Node>,
    #[serde(default)]
    pub samplers: Vec<Sampler>,
    #[serde(default)]
    pub scene: usize,
    #[serde(default)]
    pub scenes: Vec<Scene>,
    #[serde(default)]
    pub skins: Vec<Skin>,
    #[serde(default)]
    pub textures: Vec<Texture>,
}

impl Gltf {
    fn load(bytes: &[u8]) -> serde_json::Result<Self> {
        serde_json::from_slice(bytes)
    }
}

pub struct Glb {
    pub gltf: Gltf,
    pub buffer: Vec<u8>,
}

impl Glb {
    fn get_u32(bytes: &mut impl Iterator<Item = u8>) -> u32 {
        *bytemuck::from_bytes::<u32>(&bytes.take(4).collect::<Vec<u8>>())
    }

    fn get(bytes: &mut impl Iterator<Item = u8>, length: usize) -> Vec<u8> {
        bytes.take(length).collect()
    }

    pub fn load(bytes: &[u8]) -> serde_json::Result<Self> {
        let mut bytes = bytes.iter().copied();

        let magic = Self::get_u32(&mut bytes);
        if magic != 0x46546C67 {
            panic!("Malformed GLB");
        }

        let version = Self::get_u32(&mut bytes);
        if version != 2 {
            panic!("Aetheria only supports glTF 2.0");
        }

        let _length = Self::get_u32(&mut bytes);

        let gltf_length = Self::get_u32(&mut bytes);
        let gltf_type = Self::get_u32(&mut bytes);
        if gltf_type != 0x4E4F534A {
            panic!("Malformed GLB");
        }

        let gltf_bytes: Vec<u8> = Self::get(&mut bytes, gltf_length as usize);
        let gltf = Gltf::load(&gltf_bytes)?;

        let mut buffer = Vec::new();
        if !bytes.is_empty() {
            let buffer_length = Self::get_u32(&mut bytes);
            let buffer_type = Self::get_u32(&mut bytes);
            if buffer_type != 0x004E4942 {
                panic!("Malformed GLB");
            }

            buffer = bytes.take(buffer_length as usize).collect();
        }

        Ok(Self { gltf, buffer })
    }
}

```


=== systems::render.rs

```pretty-rs
// render.rs
use ash::vk;
use assets::{Mesh, Model, ModelRegistry, ShaderRegistry, Transform, Vertex};
use bytemuck::{cast_slice, Pod, Zeroable};
use glam::{Vec3, Vec4};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex, Weak},
};
use uuid::Uuid;
use vulkan::{
    command, command::TransitionLayoutOptions, compute, Buffer, Context, Image, Pool, Set,
    SetLayout, SetLayoutBuilder, Shader, Texture,
};

use crate::{
    data::Data,
    renderer::{Pass, Renderer, RENDER_HEIGHT, RENDER_WIDTH},
    Camera, Time,
};

fn calculate_box(mesh: &Mesh, transform: &Transform) -> (Vec3, Vec3) {
    let mut min = Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
    let mut max = Vec3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);
    for vertex in &mesh.vertices {
        let v = transform.get_matrix() * Vec4::new(vertex.pos.x, vertex.pos.y, vertex.pos.z, 1.0);
        min.x = min.x.min(v.x);
        min.y = min.y.min(v.y);
        min.z = min.z.min(v.z);

        max.x = max.x.max(v.x);
        max.y = max.y.max(v.y);
        max.z = max.z.max(v.z);
    }

    return (min, max);
}

#[derive(Clone)]
pub struct RenderObject {
    pub model: Arc<Model>,
    pub transform: Transform,
}

pub trait Renderable {
    fn get_objects(&self) -> Vec<RenderObject>;
}

impl<T: Renderable> Renderable for Vec<T> {
    fn get_objects(&self) -> Vec<RenderObject> {
        self.iter()
            .flat_map(|item| item.get_objects())
            .collect::<Vec<RenderObject>>()
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
struct MeshData {
    first_index: i32,
    num_indices: i32,
    material: i32,
    _padding: [f32; 1],
    min_aabb: [f32; 3],
    _padding2: [f32; 1],
    max_aabb: [f32; 3],
    _padding3: [f32; 1],
    transform: [f32; 16],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Pod, Zeroable)]
pub struct Material {
    albedo: Vec4,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct Light {
    pub position: Vec3,
    pub strength: f32,
    pub color: Vec3,
    _padding: [f32; 1],
}

impl Light {
    pub fn new(position: Vec3, strength: f32, color: Vec3) -> Self {
        Self {
            position,
            strength,
            color,
            _padding: [0.0],
        }
    }
}

pub trait Emissive {
    fn get_lights(&self, data: &Data) -> Vec<Light>;
}

pub struct System {
    texture: Texture,

    frame_layout: SetLayout,
    frame_pool: Pool,
    frame_set: Set,

    geometry_layout: SetLayout,
    geometry_pool: Pool,
    geometry_set: Set,
    pipeline: compute::Pipeline,

    renderables: Vec<Weak<Mutex<dyn Renderable>>>,
    lights: Vec<Weak<Mutex<dyn Emissive>>>,
}

impl System {
    pub fn new(
        ctx: &Context,
        shader_registry: &mut ShaderRegistry,
        camera: &Camera,
        time: &Time,
    ) -> Result<Self, vk::Result> {
        let image = Image::new(
            &ctx,
            RENDER_WIDTH,
            RENDER_HEIGHT,
            vk::Format::R8G8B8A8_UNORM,
            vk::ImageUsageFlags::STORAGE | vk::ImageUsageFlags::SAMPLED,
        )?;
        let texture =
            Texture::from_image(&ctx, image, vk::Filter::NEAREST, vk::Filter::NEAREST, true)?;

        let frame_layout = SetLayoutBuilder::new(&ctx.device)
            .add(vk::DescriptorType::UNIFORM_BUFFER)
            .add(vk::DescriptorType::UNIFORM_BUFFER)
            .build()?;
        let mut frame_pool = Pool::new(ctx.device.clone(), frame_layout.clone(), 1)?;
        let frame_set = frame_pool.allocate()?;
        frame_set.update_buffer(&ctx.device, 0, &camera.buffer);
        frame_set.update_buffer(&ctx.device, 1, &time.buffer);

        let geometry_layout = SetLayoutBuilder::new(&ctx.device)
            .add(vk::DescriptorType::STORAGE_IMAGE)
            .add(vk::DescriptorType::STORAGE_BUFFER)
            .add(vk::DescriptorType::STORAGE_BUFFER)
            .add(vk::DescriptorType::STORAGE_BUFFER)
            .add(vk::DescriptorType::STORAGE_BUFFER)
            .add(vk::DescriptorType::STORAGE_BUFFER)
            .build()?;
        let mut geometry_pool = Pool::new(ctx.device.clone(), geometry_layout.clone(), 1)?;
        let geometry_set = geometry_pool.allocate()?;
        geometry_set.update_texture(&ctx.device, 0, &texture, vk::ImageLayout::GENERAL);

        let shader: Arc<Shader> = shader_registry.load(&ctx.device, "test.comp.glsl");
        let pipeline = compute::Pipeline::new(
            &ctx.device,
            shader.clone(),
            &[frame_layout.clone(), geometry_layout.clone()],
        )?;

        Ok(Self {
            texture,
            frame_layout,
            frame_set,
            frame_pool,
            geometry_layout,
            geometry_pool,
            geometry_set,
            pipeline,
            renderables: Vec::new(),
            lights: Vec::new(),
        })
    }

    pub fn set_geometry(&self, data: &Data, renderer: &Renderer, model_registry: &ModelRegistry) {
        let objects = self
            .renderables
            .iter()
            .filter_map(|renderable| renderable.upgrade())
            .flat_map(|renderable| renderable.lock().unwrap().get_objects())
            .collect::<Vec<RenderObject>>();

        let lights = self
            .lights
            .iter()
            .filter_map(|emissive| emissive.upgrade())
            .flat_map(|emissive| emissive.lock().unwrap().get_lights(data).clone())
            .collect::<Vec<Light>>();

        let mut meshes: Vec<MeshData> = Vec::new();
        let mut vertices: Vec<Vertex> = Vec::new();
        let mut indices: Vec<i32> = Vec::new();
        let mut materials: Vec<Material> = Vec::new();

        let mut mesh_to_index: HashMap<Uuid, i32> = HashMap::new();

        for mesh in model_registry
            .get_models()
            .iter()
            .flat_map(|model| &model.meshes)
        {
            mesh_to_index.insert(mesh.id, indices.len() as i32);
            indices.append(
                &mut mesh
                    .indices
                    .iter()
                    .copied()
                    .map(|index| index as i32 + vertices.len() as i32)
                    .collect::<Vec<i32>>(),
            );
            vertices.append(&mut mesh.vertices.clone());
        }

        for (i, (mesh, transform)) in objects
            .iter()
            .flat_map(|object| {
                object
                    .model
                    .meshes
                    .iter()
                    .map(|mesh| (mesh, object.transform.clone()))
            })
            .enumerate()
        {
            let transform = transform.combine(&mesh.transform);
            let (min_aabb, max_aabb) = calculate_box(&mesh, &transform);

            let mesh_data = MeshData {
                first_index: *mesh_to_index
                    .get(&mesh.id)
                    .expect("Can't find index in mesh_to_index"),
                num_indices: mesh.indices.len() as i32,
                material: i as i32,
                transform: transform.get_matrix().to_cols_array(),
                min_aabb: min_aabb.to_array(),
                max_aabb: max_aabb.to_array(),
                ..Default::default()
            };
            meshes.push(mesh_data);
            materials.push(Material { albedo: mesh.color });
        }

        let mut mesh_data = cast_slice::<i32, u8>(&[meshes.len() as i32, 0, 0, 0]).to_vec();
        mesh_data.append(&mut cast_slice::<MeshData, u8>(&meshes).to_vec());

        let vertex_buffer = Buffer::new(
            &renderer,
            cast_slice::<Vertex, u8>(&vertices),
            vk::BufferUsageFlags::STORAGE_BUFFER,
        )
        .unwrap();
        let indices = indices
            .iter()
            .copied()
            .flat_map(|index| [index, 0, 0, 0])
            .collect::<Vec<i32>>();
        let index_buffer = Buffer::new(
            &renderer,
            cast_slice::<i32, u8>(&indices),
            vk::BufferUsageFlags::STORAGE_BUFFER,
        )
        .unwrap();
        let mesh_buffer =
            Buffer::new(&renderer, mesh_data, vk::BufferUsageFlags::STORAGE_BUFFER).unwrap();
        let material_buffer = Buffer::new(
            &renderer,
            cast_slice::<Material, u8>(&materials),
            vk::BufferUsageFlags::STORAGE_BUFFER,
        )
        .unwrap();

        let mut light_data = cast_slice::<Light, u8>(&lights).to_vec();
        let mut light_buffer = cast_slice::<i32, u8>(&[lights.len() as i32, 0, 0, 0]).to_vec();
        light_buffer.append(&mut light_data);
        let light_buffer = Buffer::new(
            &renderer,
            light_buffer,
            vk::BufferUsageFlags::STORAGE_BUFFER,
        )
        .unwrap();

        self.geometry_set
            .update_buffer(&renderer.device, 1, &vertex_buffer);
        self.geometry_set
            .update_buffer(&renderer.device, 2, &index_buffer);
        self.geometry_set
            .update_buffer(&renderer.device, 3, &mesh_buffer);
        self.geometry_set
            .update_buffer(&renderer.device, 4, &material_buffer);
        self.geometry_set
            .update_buffer(&renderer.device, 5, &light_buffer);
    }

    pub fn get_texture(&self) -> &'_ Texture {
        &self.texture
    }

    pub fn add<T: Renderable + Sized + 'static>(&mut self, renderable: Arc<Mutex<T>>) {
        self.renderables
            .push(Arc::downgrade(&(renderable as Arc<Mutex<dyn Renderable>>)));
    }

    pub fn add_light<T: Emissive + Sized + 'static>(&mut self, emissive: Arc<Mutex<T>>) {
        self.lights
            .push(Arc::downgrade(&(emissive as Arc<Mutex<dyn Emissive>>)));
    }
}

impl Pass for System {
    fn record(&self, cmd: command::BufferBuilder) -> command::BufferBuilder {
        cmd.transition_image_layout(
            &self.texture.image,
            &TransitionLayoutOptions {
                old: vk::ImageLayout::UNDEFINED,
                new: vk::ImageLayout::GENERAL,
                source_access: vk::AccessFlags::NONE,
                destination_access: vk::AccessFlags::SHADER_WRITE,
                source_stage: vk::PipelineStageFlags::TOP_OF_PIPE,
                destination_stage: vk::PipelineStageFlags::COMPUTE_SHADER,
            },
        )
        .bind_compute_pipeline(self.pipeline.clone())
        .bind_descriptor_set(0, &self.frame_set)
        .bind_descriptor_set(1, &self.geometry_set)
        .dispatch(
            RENDER_WIDTH / 16,
            (RENDER_HEIGHT as f32 / 16.0).ceil() as u32,
            1,
        )
    }
}

```


=== assets::lib.rs

```pretty-rs
// lib.rs
use ash::vk;
use bytemuck::{cast_slice, Pod, Zeroable};
use glam::{Mat4, Quat, Vec2, Vec3, Vec4};
use std::{
    collections::HashMap,
    path::Path,
    sync::{Arc, Weak},
};
use uuid::Uuid;
use vulkan::{buffer::Buffer, context::Context, device::Device, graphics::Shader, Texture};

pub struct ShaderRegistry {
    registry: HashMap<String, Weak<Shader>>,
}

impl ShaderRegistry {
    pub fn new() -> Self {
        Self {
            registry: HashMap::new(),
        }
    }

    pub fn load(&mut self, device: &Device, path: &str) -> Arc<Shader> {
        let registry_value = self
            .registry
            .get(&path.to_owned())
            .map(|weak| weak.upgrade())
            .flatten();

        match registry_value {
            Some(value) => value,
            None => {
                let spv = Path::new("assets/shaders/compiled")
                    .join(path)
                    .with_extension("spv");
                let stage = match spv
                    .file_stem()
                    .unwrap()
                    .to_str()
                    .unwrap()
                    .split(".")
                    .last()
                    .unwrap()
                {
                    "vert" => vk::ShaderStageFlags::VERTEX,
                    "frag" => vk::ShaderStageFlags::FRAGMENT,
                    "comp" => vk::ShaderStageFlags::COMPUTE,
                    shader_type => panic!("Unexpected shader type: {}", shader_type),
                };
                let code = std::fs::read(spv)
                    .ok()
                    .expect(&format!("Cannot find file: {}", path));
                let shader = Arc::new(Shader::new(device, &code, stage).unwrap());
                self.registry
                    .insert(path.to_owned(), Arc::downgrade(&shader));
                shader
            }
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable, Default)]
pub struct Vertex {
    pub pos: Vec3,
    pub _padding: f32,
    pub normal: Vec3,
    pub _padding2: f32,
}

pub struct Model {
    pub meshes: Vec<Mesh>,
}

pub struct Mesh {
    pub id: Uuid,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub color: Vec4,
    pub transform: Transform,
}

#[derive(Clone, Debug)]
pub struct Transform {
    pub translation: Vec3,
    pub rotation: Quat,
    pub scale: Vec3,
}

impl Transform {
    pub const IDENTITY: Self = Self {
        translation: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        scale: Vec3::ONE,
    };

    pub fn get_matrix(&self) -> Mat4 {
        Mat4::from_scale_rotation_translation(
            self.scale,
            self.rotation.normalize(),
            self.translation,
        )
    }

    pub fn from_matrix(matrix: &Mat4) -> Self {
        let (scale, rotation, translation) = matrix.to_scale_rotation_translation();
        Self {
            translation,
            rotation,
            scale,
        }
    }

    pub fn combine(&self, rhs: &Self) -> Self {
        Self {
            translation: self.translation + rhs.translation,
            rotation: self.rotation * rhs.rotation,
            scale: self.scale * rhs.scale,
        }
    }
}

impl Default for Transform {
    fn default() -> Self {
        Self {
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        }
    }
}

pub struct ModelRegistry {
    registry: HashMap<String, Weak<Model>>,
}

impl ModelRegistry {
    pub fn new() -> Self {
        Self {
            registry: HashMap::new(),
        }
    }

    pub fn get_models(&self) -> Vec<Arc<Model>> {
        self.registry
            .values()
            .filter_map(|weak| weak.upgrade())
            .collect()
    }

    pub fn load(&mut self, path: &str) -> Arc<Model> {
        fn get_transform(node: &gltf::Node) -> Mat4 {
            if let Some(matrix) = node.matrix {
                Mat4::from_cols_array(&matrix)
            } else {
                let scale = node
                    .scale
                    .map(|arr| Vec3::from_array(arr))
                    .unwrap_or(Vec3::ONE);
                let rotation = node
                    .rotation
                    .map(|arr| Quat::from_array(arr))
                    .unwrap_or(Quat::IDENTITY);
                let translation = node
                    .translation
                    .map(|arr| Vec3::from_array(arr))
                    .unwrap_or(Vec3::ZERO);
                Mat4::from_scale_rotation_translation(scale, rotation, translation)
            }
        }

        fn get_meshes(glb: &gltf::Glb, node: &gltf::Node, parent_transform: Mat4) -> Vec<Mesh> {
            let transform = get_transform(node) * parent_transform;

            let mut meshes = match node.mesh {
                None => Vec::new(),
                Some(mesh) => {
                    let mesh = &glb.gltf.meshes[mesh];
                    mesh.primitives
                        .iter()
                        .map(|primitive| {
                            let color = primitive
                                .material
                                .map(|material| &glb.gltf.materials[material])
                                .and_then(|material| material.pbr.base_color_factor)
                                .map(|arr| Vec4::from_array(arr))
                                .unwrap_or(Vec4::ONE);

                            let indices = primitive.get_indices_data(glb).expect("No indicies");
                            let positions = primitive
                                .get_attribute_data(glb, "POSITION")
                                .expect("No positions");
                            let positions = bytemuck::cast_slice::<u8, Vec3>(&positions)
                                .iter()
                                .map(|position| *position * 100.0)
                                .collect::<Vec<Vec3>>();
                            let normals = primitive
                                .get_attribute_data(glb, "NORMAL")
                                .expect("No normals");
                            let normals = bytemuck::cast_slice::<u8, Vec3>(&normals);
                            let vertices: Vec<Vertex> = std::iter::zip(positions, normals)
                                .map(|(pos, normal)| Vertex {
                                    pos,
                                    normal: *normal,
                                    ..Default::default()
                                })
                                .collect();
                            Mesh {
                                id: Uuid::new_v4(),
                                indices,
                                vertices,
                                color,
                                transform: Transform::from_matrix(&transform),
                            }
                        })
                        .collect()
                }
            };

            meshes.append(
                &mut node
                    .children
                    .iter()
                    .map(|child| &glb.gltf.nodes[*child])
                    .flat_map(|child| get_meshes(glb, child, transform))
                    .collect(),
            );

            meshes
        }

        let registry_value = self
            .registry
            .get(&path.to_owned())
            .map(|weak| weak.upgrade())
            .flatten();

        match registry_value {
            Some(value) => value,
            None => {
                let glb_path = Path::new("assets/meshes").join(path);
                println!("Loading: {}", glb_path.display());

                let glb = gltf::Glb::load(&std::fs::read(glb_path).unwrap()).unwrap();
                let scene = &glb.gltf.scenes[glb.gltf.scene];
                let meshes = scene
                    .nodes
                    .iter()
                    .map(|node| &glb.gltf.nodes[*node])
                    .flat_map(|node| get_meshes(&glb, node, Mat4::IDENTITY))
                    .collect();
                let model = Model { meshes };

                let model = Arc::new(model);
                self.registry
                    .insert(path.to_owned(), Arc::downgrade(&model));
                model
            }
        }
    }
}

pub struct TextureRegistry {
    registry: HashMap<String, Weak<Texture>>,
}

impl TextureRegistry {
    pub fn new() -> Self {
        Self {
            registry: HashMap::new(),
        }
    }

    pub fn get_meshes(&self) -> Vec<Arc<Texture>> {
        self.registry
            .values()
            .filter_map(|weak| weak.upgrade())
            .collect()
    }

    pub fn load(&mut self, ctx: &mut Context, path: &str, normalized_uv: bool) -> Arc<Texture> {
        let registry_value = self
            .registry
            .get(&path.to_owned())
            .map(|weak| weak.upgrade())
            .flatten();

        match registry_value {
            Some(value) => value,
            None => {
                let texture = Path::new("assets/textures/compiled").join(path);
                println!("Loading: {}", texture.display());

                let texture = Arc::new(
                    Texture::new(
                        ctx,
                        &std::fs::read(texture).expect("Failed to read texture"),
                        normalized_uv,
                    )
                    .expect("Failed to read texture"),
                );

                self.registry
                    .insert(path.to_owned(), Arc::downgrade(&texture));
                texture
            }
        }
    }
}

```


=== vulkan::allocator.rs

```pretty-rs
// allocator.rs
use crate::{Buffer, Device, Instance};
use ash::vk;
use std::{
    ffi::c_void,
    fmt::{Debug, Display},
    sync::Arc,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Region {
    size: usize,
    offset: usize,
}

impl Display for Region {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}-{}", self.offset, self.offset + self.size)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Allocation {
    id: usize,
    region: Region,
}
// Vulkan calls these memory types
#[derive(Clone, Debug)]
pub struct Heap {
    size: usize,
    properties: vk::MemoryPropertyFlags,
    memory: vk::DeviceMemory,
    allocations: Vec<Allocation>,
}

impl Display for Heap {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}: ", self.properties)?;
        for allocation in &self.allocations {
            write!(f, "{}, ", allocation.region)?;
        }

        Ok(())
    }
}

pub struct Allocator {
    device: Arc<Device>,
    heaps: Vec<Heap>,
    to_free: Vec<usize>,
    next_id: usize,
}

impl Debug for Allocator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Allocator")
            .field("heaps", &self.heaps)
            .field("next_id", &self.next_id)
            .finish()
    }
}

impl Allocator {
    pub fn new(instance: &Instance, device: Arc<Device>) -> Result<Self, vk::Result> {
        let properties =
            unsafe { instance.get_physical_device_memory_properties(*device.physical) };
        let heaps = &properties.memory_types[0..properties.memory_type_count as usize];
        let heaps = heaps
            .iter()
            .enumerate()
            .map(|(i, heap)| {
                let alloc_info = vk::MemoryAllocateInfo::builder()
                    .allocation_size(32 * 1024 * 1024) // 32MiB
                    .memory_type_index(i as u32);
                let memory = unsafe {
                    device
                        .allocate_memory(&alloc_info, None)
                        .expect("Failed to allocate memory")
                };

                Heap {
                    size: properties.memory_heaps[heap.heap_index as usize].size as usize,
                    properties: heap.property_flags,
                    memory,
                    allocations: Vec::new(),
                }
            })
            .collect::<Vec<Heap>>();
        Ok(Self {
            device,
            heaps,
            to_free: Vec::new(),
            next_id: 0,
        })
    }

    fn find_region(
        size: usize,
        alignment: usize,
        occupied: Vec<Region>,
        end: usize,
    ) -> Option<Region> {
        let mut points = vec![0_usize];
        for region in occupied {
            points.push(region.offset);
            points.push(region.offset + region.size);
        }
        points.push(end);

        let free = points
            .chunks_exact(2)
            .map(|points| {
                let from = points[0];
                let to = points[1];
                Region {
                    offset: from + (from % alignment),
                    size: to - (from + (from % alignment)),
                }
            })
            .collect::<Vec<Region>>();

        for region in free {
            if region.size > size {
                return Some(Region {
                    size,
                    offset: region.offset,
                });
            }
        }

        None
    }

    fn allocate_from_requirements(
        &mut self,
        requirements: vk::MemoryRequirements,
        properties: vk::MemoryPropertyFlags,
    ) -> (vk::DeviceMemory, Allocation) {
        let (_, heap) = self
            .heaps
            .iter_mut()
            .enumerate()
            .filter(|(i, heap)| {
                heap.properties.contains(properties)
                    && (requirements.memory_type_bits & (1 << i)) != 0
            })
            .next()
            .expect("No suitable memory heap");

        let region = Self::find_region(
            requirements.size as usize,
            requirements.alignment as usize,
            heap.allocations
                .iter()
                .map(|alloc| alloc.region)
                .collect::<Vec<Region>>(),
            32 * 1024 * 1024,
        )
        .expect("Cannot find region in heap");

        let allocation = Allocation {
            id: self.next_id,
            region,
        };

        heap.allocations.push(allocation);
        self.next_id += 1;
        (heap.memory, allocation)
    }

    pub fn create_buffer(
        &mut self,
        create_info: &vk::BufferCreateInfo,
        properties: vk::MemoryPropertyFlags,
    ) -> Result<(vk::Buffer, Allocation), vk::Result> {
        //unsafe { self.device.device_wait_idle()? };
        let buffer = unsafe { self.device.create_buffer(create_info, None)? };
        let requirements = unsafe { self.device.get_buffer_memory_requirements(buffer) };
        let (memory, allocation) = self.allocate_from_requirements(requirements, properties);
        unsafe {
            self.device
                .bind_buffer_memory(buffer, memory, allocation.region.offset as u64)?
        };

        Ok((buffer, allocation))
    }

    pub fn create_image(
        &mut self,
        create_info: &vk::ImageCreateInfo,
        properties: vk::MemoryPropertyFlags,
    ) -> Result<(vk::Image, Allocation), vk::Result> {
        //unsafe { self.device.device_wait_idle()? };
        let image = unsafe { self.device.create_image(create_info, None)? };
        let requirements = unsafe { self.device.get_image_memory_requirements(image) };
        let (memory, allocation) = self.allocate_from_requirements(requirements, properties);
        unsafe {
            self.device
                .bind_image_memory(image, memory, allocation.region.offset as u64)?
        };

        Ok((image, allocation))
    }

    pub fn write(&self, allocation: &Allocation, bytes: &[u8]) -> Result<(), vk::Result> {
        if bytes.len() > allocation.region.size {
            panic!("Buffer overflow with allocation {}", allocation.id)
        }

        let heap = self
            .heaps
            .iter()
            .find(|heap| heap.allocations.contains(allocation))
            .expect(&format!("Can't find allocation with id {}", allocation.id));
        let ptr = unsafe {
            self.device.map_memory(
                heap.memory,
                allocation.region.offset as u64,
                allocation.region.size as u64,
                vk::MemoryMapFlags::empty(),
            )?
        };
        unsafe { ptr.copy_from(bytes.as_ptr() as *const c_void, bytes.len()) };

        unsafe { self.device.unmap_memory(heap.memory) };

        Ok(())
    }

    pub fn free(&mut self, allocation: &Allocation) {
        self.heaps
            .iter_mut()
            .find(|heap| heap.allocations.contains(allocation))
            .expect(&format!("Double free of allocation {}", allocation.id));

        self.to_free.push(allocation.id);
    }

    pub fn flush_frees(&mut self) {
        for allocation in &self.to_free {
            let heap = self
                .heaps
                .iter_mut()
                .find(|heap| {
                    heap.allocations
                        .iter()
                        .find(|alloc| alloc.id == *allocation)
                        .is_some()
                })
                .expect(&format!("Double free of allocation {}", allocation));

            let allocation = heap
                .allocations
                .iter()
                .find(|alloc| alloc.id == *allocation)
                .unwrap();

            heap.allocations.remove(
                heap.allocations
                    .iter()
                    .position(|alloc| alloc.id == allocation.id)
                    .unwrap(),
            );
        }

        self.to_free.clear();
    }
}

```


=== vulkan::image.rs

```pretty-rs
// image.rs
use super::{
    allocator::{Allocation, Allocator},
    Buffer, Context, Device, Pool,
};
use crate::command::TransitionLayoutOptions;
use crate::Set;
use ash::vk::{self, MemoryPropertyFlags};
use std::cell::OnceCell;
use std::ops::Deref;
use std::path::Path;
use std::sync::{Arc, Mutex};

#[derive(Debug)]
pub struct Image {
    pub(crate) image: vk::Image,
    pub format: vk::Format,
    pub width: u32,
    pub height: u32,

    pub(crate) allocation: Option<Allocation>,
    allocator: Option<Arc<Mutex<Allocator>>>,
}

impl Image {
    pub fn new(
        ctx: &Context,
        width: u32,
        height: u32,
        format: vk::Format,
        usage: vk::ImageUsageFlags,
    ) -> Result<Arc<Self>, vk::Result> {
        let create_info = vk::ImageCreateInfo::builder()
            .image_type(vk::ImageType::TYPE_2D)
            .format(format)
            .extent(vk::Extent3D {
                width,
                height,
                depth: 1,
            })
            .mip_levels(1)
            .array_layers(1)
            .samples(vk::SampleCountFlags::TYPE_1)
            .tiling(vk::ImageTiling::OPTIMAL)
            .usage(usage)
            .sharing_mode(vk::SharingMode::EXCLUSIVE)
            .initial_layout(vk::ImageLayout::UNDEFINED);

        let (image, allocation) = ctx
            .allocator
            .lock()
            .unwrap()
            .create_image(&create_info, MemoryPropertyFlags::DEVICE_LOCAL)?;

        Ok(Arc::new(Self {
            image,
            format,
            width,
            height,
            allocation: Some(allocation),
            allocator: Some(ctx.allocator.clone()),
        }))
    }

    pub fn from_image(image: vk::Image, format: vk::Format, width: u32, height: u32) -> Arc<Self> {
        Arc::new(Self {
            image,
            format,
            width,
            height,
            allocation: None,
            allocator: None,
        })
    }

    pub fn create_view_without_context(
        &self,
        device: &Device,
    ) -> Result<vk::ImageView, vk::Result> {
        let create_info = vk::ImageViewCreateInfo::builder()
            .image(**self)
            .view_type(vk::ImageViewType::TYPE_2D)
            .format(self.format)
            .components(vk::ComponentMapping::default())
            .subresource_range(vk::ImageSubresourceRange {
                aspect_mask: vk::ImageAspectFlags::COLOR,
                base_mip_level: 0,
                level_count: 1,
                base_array_layer: 0,
                layer_count: 1,
            });

        unsafe { device.create_image_view(&create_info, None) }
    }

    pub fn create_view(&self, ctx: &Context) -> Result<vk::ImageView, vk::Result> {
        let aspect_mask = if self.format == vk::Format::D32_SFLOAT {
            vk::ImageAspectFlags::DEPTH
        } else {
            vk::ImageAspectFlags::COLOR
        };

        let create_info = vk::ImageViewCreateInfo::builder()
            .image(**self)
            .view_type(vk::ImageViewType::TYPE_2D)
            .format(self.format)
            .components(vk::ComponentMapping::default())
            .subresource_range(vk::ImageSubresourceRange {
                aspect_mask,
                base_mip_level: 0,
                level_count: 1,
                base_array_layer: 0,
                layer_count: 1,
            });

        unsafe { ctx.device.create_image_view(&create_info, None) }
    }

    pub fn create_sampler(
        &self,
        ctx: &Context,
        mag_filter: vk::Filter,
        min_filter: vk::Filter,
        normalized_uv: bool,
    ) -> Result<vk::Sampler, vk::Result> {
        let create_info = vk::SamplerCreateInfo::builder()
            .mag_filter(mag_filter)
            .min_filter(min_filter)
            .mipmap_mode(vk::SamplerMipmapMode::LINEAR)
            .address_mode_u(vk::SamplerAddressMode::MIRRORED_REPEAT)
            .address_mode_v(vk::SamplerAddressMode::MIRRORED_REPEAT)
            .address_mode_w(vk::SamplerAddressMode::MIRRORED_REPEAT)
            .mip_lod_bias(0.0)
            .anisotropy_enable(true)
            .max_anisotropy(ctx.device.physical.properties.limits.max_sampler_anisotropy)
            .compare_enable(false)
            .compare_op(vk::CompareOp::ALWAYS)
            .min_lod(0.0)
            .max_lod(0.0)
            .border_color(vk::BorderColor::INT_OPAQUE_BLACK)
            .unnormalized_coordinates(!normalized_uv);

        unsafe { ctx.device.create_sampler(&create_info, None) }
    }
}

impl Deref for Image {
    type Target = vk::Image;

    fn deref(&self) -> &Self::Target {
        &self.image
    }
}

impl Drop for Image {
    fn drop(&mut self) {
        if self.allocation.is_none() || self.allocator.is_none() {
            return;
        }

        self.allocator
            .take()
            .unwrap()
            .lock()
            .unwrap()
            .free(&self.allocation.unwrap())
    }
}

pub struct Texture {
    pub image: Arc<Image>,
    pub view: vk::ImageView,
    pub sampler: vk::Sampler,
}

impl Deref for Texture {
    type Target = Image;

    fn deref(&self) -> &Self::Target {
        &self.image
    }
}

impl Texture {
    pub const WHITE: OnceCell<Self> = OnceCell::new();

    pub fn new(ctx: &mut Context, bytes: &[u8], normalized_uv: bool) -> Result<Self, vk::Result> {
        let (header, data) = qoi::decode_to_vec(bytes).unwrap();

        Self::new_bytes(ctx, &data, header.width, header.height, normalized_uv)
    }

    pub fn new_bytes(
        ctx: &mut Context,
        data: &[u8],
        width: u32,
        height: u32,
        normalized_uv: bool,
    ) -> Result<Self, vk::Result> {
        let texture_buffer = Buffer::new(ctx, data, vk::BufferUsageFlags::TRANSFER_SRC)?;

        let image = Image::new(
            ctx,
            width,
            height,
            vk::Format::R8G8B8A8_SRGB,
            vk::ImageUsageFlags::TRANSFER_DST | vk::ImageUsageFlags::SAMPLED,
        )?;
        let view = image.create_view(ctx)?;
        let sampler =
            image.create_sampler(ctx, vk::Filter::NEAREST, vk::Filter::NEAREST, normalized_uv)?;

        ctx.command_pool
            .allocate()
            .unwrap()
            .begin()
            .unwrap()
            .transition_image_layout(
                &image,
                &TransitionLayoutOptions {
                    old: vk::ImageLayout::UNDEFINED,
                    new: vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                    source_access: vk::AccessFlags::empty(),
                    destination_access: vk::AccessFlags::TRANSFER_WRITE,
                    source_stage: vk::PipelineStageFlags::TOP_OF_PIPE,
                    destination_stage: vk::PipelineStageFlags::TRANSFER,
                },
            )
            .copy_buffer_to_image(&texture_buffer, &image)
            .transition_image_layout(
                &image,
                &TransitionLayoutOptions {
                    old: vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                    new: vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
                    source_access: vk::AccessFlags::TRANSFER_WRITE,
                    destination_access: vk::AccessFlags::SHADER_READ,
                    source_stage: vk::PipelineStageFlags::TRANSFER,
                    destination_stage: vk::PipelineStageFlags::FRAGMENT_SHADER,
                },
            )
            .submit()
            .unwrap();

        Ok(Self {
            image,
            view,
            sampler,
        })
    }

    pub fn from_image(
        ctx: &Context,
        image: Arc<Image>,
        mag_filter: vk::Filter,
        min_filter: vk::Filter,
        normalized_uv: bool,
    ) -> Result<Self, vk::Result> {
        let view = image.create_view(ctx)?;
        let sampler = image.create_sampler(ctx, mag_filter, min_filter, normalized_uv)?;
        Ok(Self {
            image,
            view,
            sampler,
        })
    }
}

```


=== vulkan::graphics.rs

```pretty-rs
// graphics.rs
use super::{Device, Renderpass, SetLayout};
use ash::vk;
use bytemuck::cast_slice;
use cstr::cstr;
use std::{ops::Deref, result::Result};

#[derive(Clone)]
pub struct Shader {
    module: vk::ShaderModule,
    pub stage: vk::ShaderStageFlags,
}

impl Shader {
    pub fn new(
        device: &ash::Device,
        code: &[u8],
        stage: vk::ShaderStageFlags,
    ) -> Result<Self, vk::Result> {
        let create_info = vk::ShaderModuleCreateInfo::builder().code(cast_slice(code));

        let module = unsafe { device.create_shader_module(&create_info, None)? };

        Ok(Self { module, stage })
    }

    pub fn get_stage(&self) -> vk::PipelineShaderStageCreateInfoBuilder {
        vk::PipelineShaderStageCreateInfo::builder()
            .stage(self.stage)
            .module(self.module)
            .name(cstr!("main"))
    }
}

#[derive(Clone)]
pub struct Shaders {
    pub vertex: Option<Shader>,
    pub fragment: Option<Shader>,
}

pub struct Binding {
    binding: usize,
    stride: usize,
    attributes: Vec<vk::VertexInputAttributeDescription>,
}

impl Binding {
    pub fn add_attribute(mut self, format: vk::Format) -> Self {
        let attribute = vk::VertexInputAttributeDescription::builder()
            .binding(self.binding.try_into().unwrap())
            .location(self.attributes.len().try_into().unwrap())
            .format(format)
            .offset(self.stride.try_into().unwrap())
            .build();
        self.attributes.push(attribute);
        self.stride += match format {
            vk::Format::R32G32_SFLOAT => 2 * 4,
            vk::Format::R32G32B32_SFLOAT => 3 * 4,
            vk::Format::R32G32B32A32_SFLOAT => 4 * 4,
            vk::Format::R8G8B8A8_UINT => 4 * 1,
            _ => todo!(),
        };

        self
    }
}

pub struct VertexInputBuilder {
    bindings: Vec<Binding>,
}

impl VertexInputBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_binding<F: Fn(Binding) -> Binding>(mut self, callback: F) -> Self {
        let binding = Binding {
            binding: self.bindings.len(),
            stride: 0,
            attributes: Vec::new(),
        };
        self.bindings.push(callback(binding));

        self
    }

    fn to_vertex_bindings(
        &self,
    ) -> (
        Vec<vk::VertexInputBindingDescription>,
        Vec<vk::VertexInputAttributeDescription>,
    ) {
        let bindings = self
            .bindings
            .iter()
            .enumerate()
            .map(|(i, binding)| {
                vk::VertexInputBindingDescription::builder()
                    .binding(i.try_into().unwrap())
                    .stride(binding.stride.try_into().unwrap())
                    .input_rate(vk::VertexInputRate::VERTEX)
                    .build()
            })
            .collect::<Vec<vk::VertexInputBindingDescription>>();
        let attributes = self
            .bindings
            .iter()
            .flat_map(|binding| binding.attributes.clone())
            .collect::<Vec<vk::VertexInputAttributeDescription>>();

        (bindings, attributes)
    }
}

impl Default for VertexInputBuilder {
    fn default() -> Self {
        Self {
            bindings: Vec::new(),
        }
    }
}

pub struct Pipeline {
    pub(crate) pipeline: vk::Pipeline,
    pub layout: vk::PipelineLayout,
    pub shaders: Shaders,
}

impl Pipeline {
    pub fn new(
        device: &Device,
        renderpass: &Renderpass,
        shaders: Shaders,
        extent: vk::Extent2D,
        descriptor_layouts: &[SetLayout],
        vertex_input: VertexInputBuilder,
        subpass: u32,
        depth: bool,
        cull: bool,
    ) -> Result<Self, vk::Result> {
        let vertex_stage = shaders
            .vertex
            .as_ref()
            .expect("All graphics pipelines need a vertex shader")
            .get_stage();
        let fragment_stage = shaders
            .fragment
            .as_ref()
            .expect("All graphics pipelines need a fragment shader")
            .get_stage();

        let (bindings, attributes) = vertex_input.to_vertex_bindings();
        let vertex_input = vk::PipelineVertexInputStateCreateInfo::builder()
            .vertex_binding_descriptions(&bindings)
            .vertex_attribute_descriptions(&attributes);
        let input_assembly = vk::PipelineInputAssemblyStateCreateInfo::builder()
            .topology(vk::PrimitiveTopology::TRIANGLE_LIST)
            .primitive_restart_enable(false);

        #[allow(clippy::cast_precision_loss)]
        let viewport = vk::Viewport::builder()
            .x(0.0)
            .y(0.0)
            .width(extent.width as f32)
            .height(extent.height as f32)
            .min_depth(0.0)
            .max_depth(1.0);
        let scissor = vk::Rect2D::builder()
            .offset(vk::Offset2D { x: 0, y: 0 })
            .extent(extent);
        let viewports = &[viewport.build()];
        let scissors = &[scissor.build()];
        let viewport_state = vk::PipelineViewportStateCreateInfo::builder()
            .viewports(viewports)
            .scissors(scissors);

        let rasterization_state = vk::PipelineRasterizationStateCreateInfo::builder()
            .depth_clamp_enable(false)
            .rasterizer_discard_enable(false)
            .polygon_mode(vk::PolygonMode::FILL)
            .line_width(1.0)
            .cull_mode(if cull {
                vk::CullModeFlags::BACK
            } else {
                vk::CullModeFlags::NONE
            })
            .front_face(vk::FrontFace::COUNTER_CLOCKWISE)
            .depth_bias_enable(false);

        let multisampling = vk::PipelineMultisampleStateCreateInfo::builder()
            .sample_shading_enable(false)
            .rasterization_samples(vk::SampleCountFlags::TYPE_1);

        let depth_stencil = vk::PipelineDepthStencilStateCreateInfo::builder()
            .depth_test_enable(true)
            .depth_write_enable(true)
            .depth_compare_op(vk::CompareOp::LESS)
            .depth_bounds_test_enable(false)
            .stencil_test_enable(false);

        let attachment = vk::PipelineColorBlendAttachmentState::builder()
            .color_write_mask(vk::ColorComponentFlags::RGBA)
            .blend_enable(false)
            .build();
        let attachments = &[attachment];
        let color_blending = vk::PipelineColorBlendStateCreateInfo::builder()
            .logic_op_enable(false)
            .logic_op(vk::LogicOp::COPY)
            .attachments(attachments)
            .blend_constants([0.0, 0.0, 0.0, 0.0]);

        let set_layouts: Vec<vk::DescriptorSetLayout> =
            descriptor_layouts.iter().map(|layout| **layout).collect();
        let layout_info = vk::PipelineLayoutCreateInfo::builder().set_layouts(&set_layouts);
        let layout = unsafe { device.create_pipeline_layout(&layout_info, None)? };

        let stages = &[*vertex_stage, *fragment_stage];

        let mut create_info = vk::GraphicsPipelineCreateInfo::builder()
            .stages(stages)
            .vertex_input_state(&vertex_input)
            .input_assembly_state(&input_assembly)
            .viewport_state(&viewport_state)
            .rasterization_state(&rasterization_state)
            .multisample_state(&multisampling)
            .color_blend_state(&color_blending)
            .layout(layout)
            .render_pass(**renderpass)
            .subpass(subpass);

        if depth {
            create_info = create_info.depth_stencil_state(&depth_stencil);
        }

        let pipeline = unsafe {
            device
                .create_graphics_pipelines(vk::PipelineCache::null(), &[*create_info], None)
                .expect("Graphics pipeline creation failed")[0]
        };

        Ok(Self {
            pipeline,
            layout,
            shaders,
        })
    }
}

impl Deref for Pipeline {
    type Target = vk::Pipeline;

    fn deref(&self) -> &Self::Target {
        &self.pipeline
    }
}

```


=== aetheria::renderer.rs

```pretty-rs
// renderer.rs
use ash::vk;
use std::ops::DerefMut;
use std::sync::Mutex;
use std::{ops::Deref, sync::Arc};
use tracing::info;
use vulkan::command::{self, TransitionLayoutOptions};
use vulkan::{Context, Image, Swapchain};
use winit::window::Window;

pub trait Pass {
    fn record(&self, cmd: command::BufferBuilder) -> command::BufferBuilder;
}

pub struct Renderer {
    pub(crate) ctx: Context,
    window: Arc<Window>,

    render_finished: vk::Semaphore,
    in_flight: vk::Fence,
    output_image: Option<(Arc<Image>, vk::ImageLayout)>,

    passes: Vec<Arc<Mutex<dyn Pass>>>,
}

pub const RENDER_WIDTH: u32 = 480;
pub const RENDER_HEIGHT: u32 = 270;

impl Renderer {
    pub fn new(ctx: Context, window: Arc<Window>) -> Result<Self, vk::Result> {
        let semaphore_info = vk::SemaphoreCreateInfo::builder();
        let fence_info = vk::FenceCreateInfo::builder().flags(vk::FenceCreateFlags::SIGNALED);
        let render_finished =
            unsafe { ctx.device.create_semaphore(&semaphore_info, None).unwrap() };
        let in_flight = unsafe { ctx.device.create_fence(&fence_info, None).unwrap() };

        let renderer = Self {
            ctx,
            window,
            render_finished,
            in_flight,
            output_image: None,
            passes: Vec::new(),
        };

        Ok(renderer)
    }

    unsafe fn destroy_swapchain(&mut self) {
        self.ctx.device.device_wait_idle().unwrap();

        self.ctx
            .swapchain
            .image_views
            .iter()
            .for_each(|view| self.ctx.device.destroy_image_view(*view, None));
        self.ctx
            .device
            .extensions
            .swapchain
            .as_ref()
            .unwrap()
            .destroy_swapchain(*self.ctx.swapchain, None);
    }

    pub fn recreate_swapchain(&mut self) -> Result<(), vk::Result> {
        unsafe { self.destroy_swapchain() };

        info!("Recreating swapchain");

        self.ctx.swapchain = Swapchain::new(
            &self.ctx.instance,
            &self.ctx.surface,
            &self.ctx.device,
            &self.window,
        )?;

        Ok(())
    }

    pub fn add_pass(&mut self, pass: Arc<Mutex<dyn Pass>>) {
        self.passes.push(pass);
    }

    pub fn set_output_image(&mut self, image: Arc<Image>, layout: vk::ImageLayout) {
        self.output_image = Some((image, layout));
    }

    pub fn wait_for_frame(&self) {
        unsafe {
            self.device
                .wait_for_fences(&[self.in_flight], true, u64::MAX)
                .unwrap();
        }
    }

    pub fn render(&mut self) {
        unsafe {
            let in_flight = self.in_flight.clone();

            let acquire_result = self.ctx.start_frame(in_flight);

            /*self.render_pass
                .set_geometry(&self, mesh_registry, renderables, lights);
            self.ui_pass.set_geometry(&self, &[Rectangle { origin: Vec2::new(50.0, 50.0), extent: Vec2::new(50.0, 50.0), radius: 25.0, color: Vec4::new(1.0, 0.0, 1.0, 0.3), ..Default::default() }]).expect("Failed to update UI geometry");*/

            let image_index = match acquire_result {
                Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => {
                    self.recreate_swapchain()
                        .expect("Swapchain recreation failed");
                    return;
                }
                Err(e) => panic!("{}", e),
                Ok(image_index) => image_index,
            };

            self.command_pool.clear();

            let cmd = self
                .command_pool
                .allocate()
                .unwrap()
                .begin()
                .unwrap()
                .record(|cmd| {
                    self.passes.iter().fold(cmd, |cmd, pass| {
                        cmd.record(|cmd| pass.lock().unwrap().record(cmd))
                    })
                })
                .transition_image_layout(
                    &self.output_image.as_ref().expect("No output image set").0,
                    &TransitionLayoutOptions {
                        old: self.output_image.as_ref().unwrap().1,
                        new: vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                        source_access: vk::AccessFlags::SHADER_WRITE,
                        destination_access: vk::AccessFlags::TRANSFER_READ,
                        source_stage: vk::PipelineStageFlags::COMPUTE_SHADER,
                        destination_stage: vk::PipelineStageFlags::TRANSFER,
                    },
                )
                .transition_image_layout(
                    &self.ctx.swapchain.images[image_index as usize],
                    &TransitionLayoutOptions {
                        old: vk::ImageLayout::UNDEFINED,
                        new: vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                        source_access: vk::AccessFlags::NONE,
                        destination_access: vk::AccessFlags::TRANSFER_WRITE,
                        source_stage: vk::PipelineStageFlags::TOP_OF_PIPE,
                        destination_stage: vk::PipelineStageFlags::TRANSFER,
                    },
                )
                .blit_image(
                    &self.output_image.as_ref().unwrap().0,
                    &self.ctx.swapchain.images[image_index as usize],
                    vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                    vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                    vk::ImageAspectFlags::COLOR,
                    vk::Filter::NEAREST,
                )
                .transition_image_layout(
                    &self.ctx.swapchain.images[image_index as usize],
                    &TransitionLayoutOptions {
                        old: vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                        new: vk::ImageLayout::PRESENT_SRC_KHR,
                        source_access: vk::AccessFlags::TRANSFER_WRITE,
                        destination_access: vk::AccessFlags::NONE,
                        source_stage: vk::PipelineStageFlags::TRANSFER,
                        destination_stage: vk::PipelineStageFlags::BOTTOM_OF_PIPE,
                    },
                )
                .end()
                .unwrap();

            let wait_semaphores = &[self.ctx.image_available];
            let signal_semaphores = &[self.render_finished];
            let command_buffers = &[*cmd];
            let submit_info = vk::SubmitInfo::builder()
                .wait_semaphores(wait_semaphores)
                .wait_dst_stage_mask(&[vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT])
                .command_buffers(command_buffers)
                .signal_semaphores(signal_semaphores);

            self.ctx
                .device
                .queue_submit(
                    self.ctx.device.queues.graphics.queue,
                    &[*submit_info],
                    self.in_flight,
                )
                .unwrap();

            let presentation_result = self.ctx.end_frame(image_index, self.render_finished);

            match presentation_result {
                Err(vk::Result::ERROR_OUT_OF_DATE_KHR) => self
                    .recreate_swapchain()
                    .expect("Swapchain recreation failed"),
                Err(e) => panic!("{}", e),
                Ok(_) => (),
            }
        }
    }
}

impl Deref for Renderer {
    type Target = Context;

    fn deref(&self) -> &Self::Target {
        &self.ctx
    }
}

impl DerefMut for Renderer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.ctx
    }
}

```


=== vulkan::descriptor.rs

```pretty-rs
// descriptor.rs
use super::{Buffer, Device, Texture};
use ash::vk;
use std::{collections::HashMap, ops::Deref, result::Result, sync::Arc};

#[derive(Clone, Copy)]
pub struct Binding {
    pub(crate) binding: vk::DescriptorSetLayoutBinding,
}

impl Binding {
    fn new(index: usize, descriptor_type: vk::DescriptorType) -> Self {
        let binding = vk::DescriptorSetLayoutBinding::builder()
            .binding(index.try_into().unwrap())
            .descriptor_type(descriptor_type)
            .descriptor_count(1)
            .stage_flags(vk::ShaderStageFlags::ALL)
            .build();

        Self { binding }
    }
}

impl Deref for Binding {
    type Target = vk::DescriptorSetLayoutBinding;

    fn deref(&self) -> &Self::Target {
        &self.binding
    }
}

pub struct SetLayoutBuilder<'a> {
    device: &'a Device,
    bindings: Vec<Binding>,
}

impl<'a> SetLayoutBuilder<'a> {
    pub const fn new(device: &'a Device) -> Self {
        Self {
            device,
            bindings: Vec::new(),
        }
    }

    pub fn add(mut self, descriptor_type: vk::DescriptorType) -> Self {
        self.bindings
            .push(Binding::new(self.bindings.len(), descriptor_type));

        self
    }

    pub fn build(self) -> Result<SetLayout, vk::Result> {
        let bindings: Vec<vk::DescriptorSetLayoutBinding> =
            self.bindings.iter().map(|binding| **binding).collect();
        let create_info = vk::DescriptorSetLayoutCreateInfo::builder().bindings(&bindings);

        let layout = unsafe {
            self.device
                .create_descriptor_set_layout(&create_info, None)?
        };

        Ok(SetLayout {
            layout,
            binding_types: self
                .bindings
                .iter()
                .map(|binding| binding.descriptor_type)
                .collect(),
        })
    }
}

#[derive(Clone)]
pub struct SetLayout {
    pub(crate) layout: vk::DescriptorSetLayout,
    pub binding_types: Vec<vk::DescriptorType>,
}

impl Deref for SetLayout {
    type Target = vk::DescriptorSetLayout;

    fn deref(&self) -> &Self::Target {
        &self.layout
    }
}

#[derive(Clone)]
pub struct Set {
    pub(crate) set: vk::DescriptorSet,
    binding_types: Vec<vk::DescriptorType>,
}

impl Set {
    pub fn update_buffer(&self, device: &Device, binding: u32, buffer: &Buffer) {
        let buffer_info = vk::DescriptorBufferInfo::builder()
            .buffer(**buffer)
            .offset(0)
            .range(buffer.size.try_into().unwrap());

        let buffer_infos = &[*buffer_info];
        let write_info = vk::WriteDescriptorSet::builder()
            .dst_set(**self)
            .dst_binding(binding)
            .dst_array_element(0)
            .descriptor_type(self.binding_types[binding as usize])
            .buffer_info(buffer_infos);

        let descriptor_writes = &[*write_info];

        unsafe { device.update_descriptor_sets(descriptor_writes, &[]) };
    }

    pub fn update_texture(
        &self,
        device: &Device,
        binding: u32,
        texture: &Texture,
        layout: vk::ImageLayout,
    ) {
        let image_info = vk::DescriptorImageInfo::builder()
            .sampler(texture.sampler)
            .image_view(texture.view)
            .image_layout(layout);

        let image_infos = &[*image_info];
        let write_info = vk::WriteDescriptorSet::builder()
            .dst_set(**self)
            .dst_binding(binding)
            .dst_array_element(0)
            .descriptor_type(self.binding_types[binding as usize])
            .image_info(image_infos);

        let descriptor_writes = &[*write_info];

        unsafe { device.update_descriptor_sets(descriptor_writes, &[]) };
    }
}

impl Deref for Set {
    type Target = vk::DescriptorSet;

    fn deref(&self) -> &Self::Target {
        &self.set
    }
}

pub struct Pool {
    pub(crate) pool: vk::DescriptorPool,
    device: Arc<Device>,
    layout: SetLayout,
    sets: Vec<Set>,
}

impl Pool {
    pub fn new(
        device: Arc<Device>,
        layout: SetLayout,
        capacity: usize,
    ) -> Result<Self, vk::Result> {
        let descriptor_types: Vec<vk::DescriptorType> = layout.binding_types.clone();

        let mut descriptor_type_amounts: HashMap<vk::DescriptorType, usize> = HashMap::new();
        for descriptor_type in &descriptor_types {
            match descriptor_type_amounts.get_mut(descriptor_type) {
                Some(amount) => {
                    *amount += 1;
                }
                None => {
                    descriptor_type_amounts.insert(*descriptor_type, 1);
                }
            }
        }

        let pool_sizes: Vec<vk::DescriptorPoolSize> = descriptor_type_amounts
            .into_iter()
            .map(|(descriptor_type, amount)| {
                vk::DescriptorPoolSize::builder()
                    .ty(descriptor_type)
                    .descriptor_count((amount * capacity).try_into().unwrap())
                    .build()
            })
            .collect();

        let create_info = vk::DescriptorPoolCreateInfo::builder()
            .max_sets(capacity.try_into().unwrap())
            .pool_sizes(&pool_sizes);

        let pool = unsafe { device.create_descriptor_pool(&create_info, None)? };

        Ok(Self {
            pool,
            device,
            layout,
            sets: Vec::new(),
        })
    }

    pub fn allocate(&mut self) -> Result<Set, vk::Result> {
        let set_layouts = &[*self.layout];
        let allocate_info = vk::DescriptorSetAllocateInfo::builder()
            .descriptor_pool(**self)
            .set_layouts(set_layouts);

        let set = unsafe { self.device.allocate_descriptor_sets(&allocate_info)?[0] };

        Ok(Set {
            set,
            binding_types: self.layout.binding_types.clone(),
        })
    }
}

impl Deref for Pool {
    type Target = vk::DescriptorPool;

    fn deref(&self) -> &Self::Target {
        &self.pool
    }
}

```


=== vulkan::device.rs

```pretty-rs
// device.rs
use super::{Instance, Surface};
use ash::{extensions::khr, vk};
use bytemuck::cast_slice;
use std::{collections::HashSet, ffi::CStr, ops::Deref, result::Result};
use tracing::info;

pub struct Queue {
    pub queue: vk::Queue,
    pub index: u32,
}

impl Queue {
    const fn new(queue: vk::Queue, index: u32) -> Self {
        Self { queue, index }
    }
}

impl Deref for Queue {
    type Target = vk::Queue;

    fn deref(&self) -> &Self::Target {
        &self.queue
    }
}

pub struct Queues {
    pub graphics: Queue,
    pub present: Queue,
}

pub struct Extensions {
    pub swapchain: Option<khr::Swapchain>,
}

impl Extensions {
    fn load(instance: &ash::Instance, device: &ash::Device, available: &[&CStr]) -> Self {
        Self {
            swapchain: available
                .iter()
                .find(|ext| **ext == khr::Swapchain::name())
                .map(|_| khr::Swapchain::new(instance, device)),
        }
    }
}

pub struct Device {
    pub(crate) device: ash::Device,
    pub physical: super::instance::PhysicalDevice,
    pub queues: Queues,
    pub extensions: Extensions,
}

impl Device {
    pub unsafe fn new(instance: &Instance, surface: &Surface) -> Result<Self, vk::Result> {
        let physicals = instance.get_physical_devices()?;
        let physical = physicals
            .first()
            .cloned()
            .expect("No device supporting vulkan found");

        let mut features = vk::PhysicalDeviceFeatures::default();
        features.sampler_anisotropy = physical.features.sampler_anisotropy;

        let (graphics_family_index, _graphics_family) = physical
            .queue_families
            .iter()
            .enumerate()
            .find(|(_, family)| family.queue_flags.intersects(vk::QueueFlags::GRAPHICS))
            .expect("No graphics queue family");

        let (present_family_index, _present_family) = physical
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

        info!("Found graphics family at index {}", graphics_family_index);
        info!("Found present family at index {}", present_family_index);

        let queue_family_indices = [graphics_family_index, present_family_index];
        let unique_queue_family_indices: HashSet<usize> = HashSet::from_iter(queue_family_indices);

        let queue_priorities = [1.0];
        let queue_create_infos: Vec<vk::DeviceQueueCreateInfo> = unique_queue_family_indices
            .iter()
            .map(|index| {
                vk::DeviceQueueCreateInfo::builder()
                    .queue_family_index((*index).try_into().unwrap())
                    .queue_priorities(&queue_priorities)
                    .build()
            })
            .collect();

        let available_layers = instance.enumerate_device_layer_properties(physical.physical)?;
        let available_extensions =
            instance.enumerate_device_extension_properties(physical.physical)?;

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

        info!("Using device layers: {:?}", wanted_layers);
        info!("Using device extesions: {:?}", wanted_extensions);

        let wanted_layers_raw: Vec<*const i8> =
            wanted_layers.iter().map(|name| name.as_ptr()).collect();
        let wanted_extensions_raw: Vec<*const i8> =
            wanted_extensions.iter().map(|name| name.as_ptr()).collect();

        let create_info = vk::DeviceCreateInfo::builder()
            .queue_create_infos(&queue_create_infos)
            .enabled_layer_names(&wanted_layers_raw)
            .enabled_extension_names(&wanted_extensions_raw)
            .enabled_features(&features);

        let device = unsafe { instance.create_device(physical.physical, &create_info, None)? };

        info!("Created vulkan device");

        let graphics = device.get_device_queue(graphics_family_index.try_into().unwrap(), 0);
        let graphics = Queue::new(graphics, graphics_family_index.try_into().unwrap());

        let present = device.get_device_queue(present_family_index.try_into().unwrap(), 0);
        let present = Queue::new(present, present_family_index.try_into().unwrap());

        Ok(Self {
            extensions: Extensions::load(instance, &device, &available_extension_names),
            device,
            physical,
            queues: Queues { graphics, present },
        })
    }
}

impl Deref for Device {
    type Target = ash::Device;

    fn deref(&self) -> &Self::Target {
        &self.device
    }
}

fn get_wanted_extensions() -> Vec<&'static CStr> {
    vec![khr::Swapchain::name()]
}

```


=== vulkan::instance.rs

```pretty-rs
// instance.rs
use ash::{extensions::khr, vk};
use bytemuck::cast_slice;
use cstr::cstr;
use std::{clone::Clone, ffi::CStr, ops::Deref, result::Result};
use tracing::info;

#[derive(Debug, Clone)]
pub struct PhysicalDeviceProperties {
    properties: vk::PhysicalDeviceProperties,
    pub device_name: String,
}

impl PhysicalDeviceProperties {
    fn new(properties: &vk::PhysicalDeviceProperties) -> Self {
        let device_name_raw: &CStr =
            CStr::from_bytes_until_nul(cast_slice(&properties.device_name)).unwrap();
        let device_name = device_name_raw.to_str().unwrap().to_owned();

        Self {
            properties: *properties,
            device_name,
        }
    }
}

impl Deref for PhysicalDeviceProperties {
    type Target = vk::PhysicalDeviceProperties;
    fn deref(&self) -> &Self::Target {
        &self.properties
    }
}

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

impl Deref for PhysicalDevice {
    type Target = vk::PhysicalDevice;

    fn deref(&self) -> &Self::Target {
        &self.physical
    }
}

#[derive(Clone)]
pub struct Extensions {
    pub surface: Option<khr::Surface>,
    pub xlib_surface: Option<khr::XlibSurface>,
    pub win32_surface: Option<khr::Win32Surface>,
}

impl Extensions {
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
            win32_surface: available
                .iter()
                .find(|ext| **ext == khr::Win32Surface::name())
                .map(|_| khr::Win32Surface::new(entry, instance)),
        }
    }
}

#[derive(Clone)]
pub struct Instance {
    instance: ash::Instance,
    pub extensions: Extensions,
}

impl Instance {
    pub fn new(entry: &ash::Entry) -> Result<Self, vk::Result> {
        let app_info = vk::ApplicationInfo::builder()
            .application_name(cstr!("aetheria"))
            .application_version(vk::make_api_version(0, 1, 0, 0))
            .engine_name(cstr!("aetheria"))
            .engine_version(vk::make_api_version(0, 1, 0, 0))
            .api_version(vk::make_api_version(0, 1, 3, 238));

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

        let instance_info = vk::InstanceCreateInfo::builder()
            .application_info(&app_info)
            .enabled_layer_names(&wanted_layers_raw)
            .enabled_extension_names(&wanted_extensions_raw);

        let instance = unsafe { entry.create_instance(&instance_info, None)? };

        Ok(Self {
            extensions: Extensions::load(entry, &instance, &available_extension_names),
            instance,
        })
    }

    pub fn get_physical_devices(&self) -> Result<Vec<PhysicalDevice>, vk::Result> {
        let physicals = unsafe { self.enumerate_physical_devices()? };
        unsafe {
            Ok(physicals
                .iter()
                .copied()
                .map(|physical| PhysicalDevice::new(self, physical))
                .collect())
        }
    }
}

impl Deref for Instance {
    type Target = ash::Instance;
    fn deref(&self) -> &Self::Target {
        &self.instance
    }
}

#[cfg(target_os = "linux")]
fn get_wanted_extensions() -> Vec<&'static CStr> {
    vec![khr::Surface::name(), khr::XlibSurface::name()]
}

#[cfg(target_os = "windows")]
fn get_wanted_extensions() -> Vec<&'static CStr> {
    vec![khr::Surface::name(), khr::Win32Surface::name()]
}

```


=== vulkan::renderpass.rs

```pretty-rs
// renderpass.rs
use super::{Device, Image};
use ash::vk;
use std::ops::Deref;

pub struct Renderpass {
    pub(crate) renderpass: vk::RenderPass,
}

impl Renderpass {
    pub fn new_render(device: &Device, color_format: vk::Format) -> Result<Self, vk::Result> {
        let color_attachment = vk::AttachmentDescription::builder()
            .format(color_format)
            .samples(vk::SampleCountFlags::TYPE_1)
            .load_op(vk::AttachmentLoadOp::CLEAR)
            .store_op(vk::AttachmentStoreOp::STORE)
            .stencil_load_op(vk::AttachmentLoadOp::DONT_CARE)
            .stencil_store_op(vk::AttachmentStoreOp::DONT_CARE)
            .initial_layout(vk::ImageLayout::UNDEFINED)
            .final_layout(vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL);

        let color_attachment_ref = vk::AttachmentReference::builder()
            .attachment(0)
            .layout(vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL);

        let depth_attachment = vk::AttachmentDescription::builder()
            .format(vk::Format::D32_SFLOAT)
            .samples(vk::SampleCountFlags::TYPE_1)
            .load_op(vk::AttachmentLoadOp::CLEAR)
            .store_op(vk::AttachmentStoreOp::DONT_CARE)
            .stencil_load_op(vk::AttachmentLoadOp::DONT_CARE)
            .stencil_store_op(vk::AttachmentStoreOp::DONT_CARE)
            .initial_layout(vk::ImageLayout::UNDEFINED)
            .final_layout(vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL);

        let depth_attachment_ref = vk::AttachmentReference::builder()
            .attachment(1)
            .layout(vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL);

        let color_attachments = &[*color_attachment_ref];
        let geometry_subpass = vk::SubpassDescription::builder()
            .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
            .color_attachments(color_attachments)
            .depth_stencil_attachment(&depth_attachment_ref);
        let grass_subpass = vk::SubpassDescription::builder()
            .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
            .color_attachments(color_attachments)
            .depth_stencil_attachment(&depth_attachment_ref);

        let dependency = vk::SubpassDependency::builder()
            .src_subpass(0)
            .dst_subpass(1)
            .src_stage_mask(vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT)
            .src_access_mask(vk::AccessFlags::COLOR_ATTACHMENT_WRITE)
            .dst_stage_mask(vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT)
            .dst_access_mask(vk::AccessFlags::COLOR_ATTACHMENT_WRITE);

        let attachments = &[*color_attachment, *depth_attachment];
        let subpasses = &[*geometry_subpass, *grass_subpass];
        let dependencies = &[*dependency];
        let create_info = vk::RenderPassCreateInfo::builder()
            .attachments(attachments)
            .subpasses(subpasses)
            .dependencies(dependencies);

        let renderpass = unsafe { device.create_render_pass(&create_info, None)? };

        Ok(Self { renderpass })
    }

    pub fn new_upscale_ui(device: &Device, color_format: vk::Format) -> Result<Self, vk::Result> {
        let color_attachment = vk::AttachmentDescription::builder()
            .format(color_format)
            .samples(vk::SampleCountFlags::TYPE_1)
            .load_op(vk::AttachmentLoadOp::CLEAR)
            .store_op(vk::AttachmentStoreOp::STORE)
            .stencil_load_op(vk::AttachmentLoadOp::DONT_CARE)
            .stencil_store_op(vk::AttachmentStoreOp::DONT_CARE)
            .initial_layout(vk::ImageLayout::UNDEFINED)
            .final_layout(vk::ImageLayout::PRESENT_SRC_KHR);

        let color_attachment_ref = vk::AttachmentReference::builder()
            .attachment(0)
            .layout(vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL);

        let color_attachments = &[*color_attachment_ref];
        let upscale_subpass = vk::SubpassDescription::builder()
            .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
            .color_attachments(color_attachments);
        let ui_subpass = vk::SubpassDescription::builder()
            .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
            .color_attachments(color_attachments);

        let dependency = vk::SubpassDependency::builder()
            .src_subpass(0)
            .dst_subpass(1)
            .src_stage_mask(vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT)
            .src_access_mask(vk::AccessFlags::COLOR_ATTACHMENT_WRITE)
            .dst_stage_mask(vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT)
            .dst_access_mask(vk::AccessFlags::COLOR_ATTACHMENT_WRITE);

        let attachments = &[*color_attachment];
        let subpasses = &[*upscale_subpass, *ui_subpass];
        let dependencies = &[*dependency];
        let create_info = vk::RenderPassCreateInfo::builder()
            .attachments(attachments)
            .subpasses(subpasses)
            .dependencies(dependencies);

        let renderpass = unsafe { device.create_render_pass(&create_info, None)? };

        Ok(Self { renderpass })
    }

    pub fn create_framebuffer(
        &self,
        device: &Device,
        width: u32,
        height: u32,
        attachments: &[vk::ImageView],
    ) -> Result<vk::Framebuffer, vk::Result> {
        let create_info = vk::FramebufferCreateInfo::builder()
            .render_pass(**self)
            .attachments(attachments)
            .width(width)
            .height(height)
            .layers(1);

        unsafe { device.create_framebuffer(&create_info, None) }
    }
}

impl Deref for Renderpass {
    type Target = vk::RenderPass;

    fn deref(&self) -> &Self::Target {
        &self.renderpass
    }
}

```


=== aetheria::ui.rs

```pretty-rs
// ui.rs
use ash::vk;
use assets::{ShaderRegistry, TextureRegistry};
use bytemuck::{cast_slice, Pod, Zeroable};
use glam::{UVec2, Vec4};
use std::sync::Arc;
use vulkan::{
    command, command::TransitionLayoutOptions, compute, Buffer, Image, Pool, Set, SetLayout,
    SetLayoutBuilder, Shader, Texture,
};
use winit::event::MouseButton;

use crate::renderer::{Pass, Renderer, RENDER_HEIGHT, RENDER_WIDTH};

#[derive(Clone, Debug, PartialEq)]
pub struct SizeConstraints {
    pub min: UVec2,
    pub max: UVec2,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    pub origin: UVec2,
    pub size: UVec2,
}

pub mod color {
    use glam::Vec4;

    pub const fn get_highlight() -> Vec4 {
        Vec4::new(0.957, 0.247, 0.369, 1.0)
    }

    pub const fn get_background() -> Vec4 {
        Vec4::new(0.094, 0.094, 0.106, 1.0)
    }

    pub const fn get_success() -> Vec4 {
        Vec4::new(0.133, 0.773, 0.369, 1.0)
    }
}

pub mod input {
    use glam::UVec2;
    use winit::event::MouseButton;

    use crate::input::Mouse;

    use super::Region;

    pub fn hovering(mouse: &Mouse, region: &Region) -> bool {
        let position = mouse.get_position();

        let min = region.origin;
        let max = region.origin + region.size;

        min.x < position.x && position.x < max.x && min.y < position.y && position.y < max.y
    }

    pub fn clicked(mouse: &Mouse, region: &Region, button: MouseButton) -> bool {
        hovering(mouse, region) && mouse.is_button_pressed(button)
    }
}

pub trait Element {
    fn layout(&mut self, constraint: SizeConstraints) -> UVec2;
    fn paint(&mut self, region: Region, scene: &mut Vec<Rectangle>);
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct Rectangle {
    pub color: Vec4,
    pub origin: UVec2,
    pub extent: UVec2,
    pub radius: u32,
    pub atlas_id: i32,
    pub _padding: [u8; 8],
}

impl Default for Rectangle {
    fn default() -> Self {
        Self {
            color: Vec4::ONE,
            origin: UVec2::ZERO,
            extent: UVec2::ONE,
            radius: 0,
            atlas_id: -1,
            _padding: [0_u8; 8],
        }
    }
}

pub struct UIPass {
    pipeline: compute::Pipeline,
    font: Arc<Texture>,
    ui_layout: SetLayout,
    ui_pool: Pool,
    ui_set: Set,
    output: Texture,
}

impl UIPass {
    pub fn new(
        renderer: &mut Renderer,
        shader_registry: &mut ShaderRegistry,
        texture_registry: &mut TextureRegistry,
        input: &Texture,
    ) -> Result<Self, vk::Result> {
        let image = Image::new(
            &renderer,
            RENDER_WIDTH,
            RENDER_HEIGHT,
            vk::Format::R8G8B8A8_UNORM,
            vk::ImageUsageFlags::STORAGE | vk::ImageUsageFlags::TRANSFER_SRC,
        )?;
        let output = Texture::from_image(
            &renderer,
            image,
            vk::Filter::NEAREST,
            vk::Filter::NEAREST,
            true,
        )?;

        let ui_layout = SetLayoutBuilder::new(&renderer.device)
            .add(vk::DescriptorType::STORAGE_IMAGE)
            .add(vk::DescriptorType::STORAGE_IMAGE)
            .add(vk::DescriptorType::COMBINED_IMAGE_SAMPLER)
            .add(vk::DescriptorType::STORAGE_BUFFER)
            .build()?;
        let mut ui_pool = Pool::new(renderer.device.clone(), ui_layout.clone(), 1)?;
        let ui_set = ui_pool.allocate()?;
        ui_set.update_texture(&renderer.device, 0, &output, vk::ImageLayout::GENERAL);
        ui_set.update_texture(&renderer.device, 1, &input, vk::ImageLayout::GENERAL);

        let font = texture_registry.load(renderer, "font.qoi", false);
        ui_set.update_texture(
            &renderer.device,
            2,
            &font,
            vk::ImageLayout::SHADER_READ_ONLY_OPTIMAL,
        );

        let shader: Arc<Shader> = shader_registry.load(&renderer.device, "ui.comp.glsl");
        let pipeline =
            compute::Pipeline::new(&renderer.device, shader.clone(), &[ui_layout.clone()])?;

        Ok(Self {
            pipeline,
            ui_layout,
            ui_pool,
            ui_set,
            font,
            output,
        })
    }

    pub fn set_geometry(
        &self,
        renderer: &Renderer,
        rectangles: &[Rectangle],
    ) -> Result<(), vk::Result> {
        let mut rectangle_data: Vec<u8> =
            cast_slice::<i32, u8>(&[rectangles.len() as i32, 0, 0, 0]).to_vec();
        rectangle_data.extend_from_slice(cast_slice::<Rectangle, u8>(rectangles));
        let rectangle_buffer = Buffer::new(
            renderer,
            rectangle_data,
            vk::BufferUsageFlags::STORAGE_BUFFER,
        )?;
        self.ui_set
            .update_buffer(&renderer.device, 3, &rectangle_buffer);

        Ok(())
    }

    pub fn get_texture(&self) -> &'_ Texture {
        &self.output
    }
}

impl Pass for UIPass {
    fn record(&self, cmd: command::BufferBuilder) -> command::BufferBuilder {
        cmd.transition_image_layout(
            &self.output.image,
            &TransitionLayoutOptions {
                old: vk::ImageLayout::UNDEFINED,
                new: vk::ImageLayout::GENERAL,
                source_access: vk::AccessFlags::NONE,
                destination_access: vk::AccessFlags::SHADER_WRITE,
                source_stage: vk::PipelineStageFlags::TOP_OF_PIPE,
                destination_stage: vk::PipelineStageFlags::COMPUTE_SHADER,
            },
        )
        .bind_compute_pipeline(self.pipeline.clone())
        .bind_descriptor_set(0, &self.ui_set)
        .dispatch(
            RENDER_WIDTH / 16,
            (RENDER_HEIGHT as f32 / 16.0).ceil() as u32,
            1,
        )
    }
}

```


=== vulkan::swapchain.rs

```pretty-rs
// swapchain.rs
use super::{Device, Image, Instance, Surface};
use ash::vk;
use std::{ops::Deref, sync::Arc};
use winit::window::Window;

#[derive(Debug)]
pub struct Swapchain {
    pub(crate) swapchain: vk::SwapchainKHR,
    pub format: vk::Format,
    pub extent: vk::Extent2D,
    pub images: Vec<Arc<Image>>,
    pub image_views: Vec<vk::ImageView>,
}

impl Swapchain {
    pub fn new(
        instance: &Instance,
        surface: &Surface,
        device: &Device,
        window: &Window,
    ) -> Result<Self, vk::Result> {
        let surface_khr = instance.extensions.surface.as_ref().unwrap();
        let swapchain_khr = device.extensions.swapchain.as_ref().unwrap();

        let capabilities = unsafe {
            surface_khr.get_physical_device_surface_capabilities(
                device.physical.physical,
                surface.surface,
            )?
        };

        let formats = unsafe {
            surface_khr
                .get_physical_device_surface_formats(device.physical.physical, surface.surface)?
        };

        let present_modes = unsafe {
            surface_khr.get_physical_device_surface_present_modes(
                device.physical.physical,
                surface.surface,
            )?
        };

        let format = formats
            .iter()
            .find(|format| {
                format.format == vk::Format::B8G8R8A8_SRGB
                    && format.color_space == vk::ColorSpaceKHR::SRGB_NONLINEAR
            })
            .unwrap_or_else(|| formats.first().unwrap());

        let present_mode = present_modes
            .iter()
            .copied()
            //.find(|present_mode| *present_mode == vk::PresentModeKHR::FIFO)
            .find(|present_mode| *present_mode == vk::PresentModeKHR::MAILBOX)
            .unwrap_or(vk::PresentModeKHR::FIFO);

        let extent = if capabilities.current_extent.width == u32::MAX {
            vk::Extent2D {
                width: window.inner_size().width,
                height: window.inner_size().height,
            }
        } else {
            capabilities.current_extent
        };

        let image_count = if capabilities.max_image_count == 0
            || capabilities.min_image_count + 1 < capabilities.max_image_count
        {
            capabilities.min_image_count + 1
        } else {
            capabilities.min_image_count
        };

        let (sharing_mode, queue_family_indices) =
            if device.queues.graphics.index == device.queues.present.index {
                (vk::SharingMode::EXCLUSIVE, Vec::new())
            } else {
                (
                    vk::SharingMode::CONCURRENT,
                    vec![device.queues.graphics.index, device.queues.present.index],
                )
            };

        let create_info = vk::SwapchainCreateInfoKHR::builder()
            .surface(surface.surface)
            .min_image_count(image_count)
            .image_format(format.format)
            .image_color_space(format.color_space)
            .image_extent(extent)
            .image_array_layers(1)
            .image_usage(vk::ImageUsageFlags::COLOR_ATTACHMENT | vk::ImageUsageFlags::TRANSFER_DST)
            .image_sharing_mode(sharing_mode)
            .queue_family_indices(&queue_family_indices)
            .pre_transform(capabilities.current_transform)
            .composite_alpha(vk::CompositeAlphaFlagsKHR::OPAQUE)
            .present_mode(present_mode)
            .clipped(true);

        let swapchain = unsafe { swapchain_khr.create_swapchain(&create_info, None)? };

        let images = unsafe { swapchain_khr.get_swapchain_images(swapchain)? };
        let images: Vec<Arc<Image>> = images
            .iter()
            .copied()
            .map(|image| Image::from_image(image, format.format, extent.width, extent.height))
            .collect();

        let image_views = images
            .iter()
            .map(|image| image.create_view_without_context(device).unwrap())
            .collect();

        Ok(Self {
            swapchain,
            format: format.format,
            extent,
            images,
            image_views,
        })
    }
}

impl Deref for Swapchain {
    type Target = vk::SwapchainKHR;

    fn deref(&self) -> &Self::Target {
        &self.swapchain
    }
}

```


=== components::craft.rs

```pretty-rs
// craft.rs
use common::item::ItemStack;

use super::components::{
    Button, Container, HAlign, HPair, Handler, Padding, Text, VAlign, VList, VPair,
};
use crate::{
    data::{inventory::Inventory, Data, Recipe},
    input::Mouse,
    ui::{self, Element},
};
use glam::Vec4;
use std::sync::{Arc, Mutex};

pub struct CraftButtonHandler<'a> {
    recipe: Recipe,
    data: Arc<Mutex<&'a mut Data>>,
}

impl Handler for CraftButtonHandler<'_> {
    fn handle(&mut self) {
        if !self
            .recipe
            .has_ingredients(&self.data.lock().unwrap().inventory)
        {
            return;
        }

        self.recipe
            .ingredients
            .iter()
            .for_each(|stack| self.data.lock().unwrap().inventory.remove(*stack));
        self.recipe
            .outputs
            .iter()
            .for_each(|stack| self.data.lock().unwrap().inventory.add(*stack));

        self.data.lock().unwrap().current_recipe = None;
    }
}

pub struct CloseHandler<'a> {
    data: Arc<Mutex<&'a mut Data>>,
}

impl Handler for CloseHandler<'_> {
    fn handle(&mut self) {
        self.data.lock().unwrap().current_recipe = None
    }
}

pub type Component<'a> = Container<
    Padding<
        VPair<VList<Text>, HPair<Button<'a, CloseHandler<'a>>, Button<'a, CraftButtonHandler<'a>>>>,
    >,
>;

impl<'a> Component<'a> {
    pub fn new(data: &'a mut Data, mouse: &'a Mouse) -> Option<Self> {
        let mut text = Vec::new();
        let color = if data
            .current_recipe
            .as_ref()?
            .has_ingredients(&data.inventory)
        {
            ui::color::get_success()
        } else {
            ui::color::get_highlight()
        };

        text.push(Text {
            color,
            content: "Ingredients".to_owned(),
        });
        data.current_recipe
            .as_ref()?
            .ingredients
            .iter()
            .for_each(|ingredient| {
                let inventory_amount = data
                    .inventory
                    .get_items()
                    .iter()
                    .find(|stack| stack.item == ingredient.item)
                    .map(|stack| stack.amount)
                    .unwrap_or(0);

                let color = if inventory_amount >= ingredient.amount {
                    ui::color::get_success()
                } else {
                    ui::color::get_highlight()
                };

                text.push(Text {
                    color,
                    content: format!(
                        "{} {}/{}",
                        ingredient.item, inventory_amount, ingredient.amount
                    ),
                })
            });
        text.push(Text {
            color: Vec4::ZERO,
            content: String::new(),
        });
        text.push(Text {
            color: ui::color::get_highlight(),
            content: "Outputs".to_owned(),
        });
        data.current_recipe
            .as_ref()?
            .outputs
            .iter()
            .for_each(|output| {
                text.push(Text {
                    color: ui::color::get_highlight(),
                    content: format!("{}", output),
                })
            });

        let text = VList {
            children: text,
            separation: 2,
            align: HAlign::Left,
        };

        let recipe = data.current_recipe.clone()?;
        let data_mutex = Arc::new(Mutex::new(data));
        let craft_handler = CraftButtonHandler {
            recipe,
            data: data_mutex.clone(),
        };
        let craft_button = Button::new(mouse, "Craft", craft_handler);
        let close_handler = CloseHandler { data: data_mutex };
        let close_button = Button::new(mouse, "Cancel", close_handler);

        let pair = VPair::new(
            text,
            HPair::new(close_button, craft_button, VAlign::Top, 4),
            HAlign::Center,
            6,
        );

        Some(Self {
            child: Padding::new_uniform(pair, 2),
            color: ui::color::get_background(),
            border_color: ui::color::get_highlight(),
            border_radius: 1,
        })
    }
}

```


=== entities::firefly.rs

```pretty-rs
// firefly.rs
use std::sync::{Arc, Mutex, Weak};

use ash::vk;
use assets::{ModelRegistry, Transform};
use glam::{Quat, Vec3};
use rand::Rng;

use super::Sun;
use crate::{
    data::{inventory::Inventory, Data},
    renderer::Renderer,
    systems::{
        interact::Interactable,
        render::{Emissive, Light, RenderObject, Renderable, System},
        Named, Positioned, Systems,
    },
    time::Time,
};
use common::item::{Item, ItemStack};

const FIREFLY_SPEED: f32 = 60.0;

pub struct Firefly {
    light: Light,
    velocity: Vec3,
    origin: Vec3,
    render: RenderObject,
    gathered: bool,
}

impl Firefly {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
        translation: Vec3,
        color: Vec3,
    ) -> Result<Arc<Mutex<Self>>, vk::Result> {
        let light = Light::new(translation, 0.0, color);

        let transform = Transform {
            translation,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        };
        let render = RenderObject {
            model: model_registry.load("firefly.glb"),
            transform,
        };

        let mut rng = rand::thread_rng();
        let velocity = Vec3::new(
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        )
        .normalize_or_zero();
        let firefly = Arc::new(Mutex::new(Self {
            light,
            velocity,
            origin: translation,
            render,
            gathered: false,
        }));

        systems.render.add(firefly.clone());
        systems.render.add_light(firefly.clone());
        systems.interact.add(firefly.clone());
        Ok(firefly)
    }

    pub fn frame_finished(&mut self, sun: &Sun, time: &Time) {
        if sun.get_theta() > (std::f32::consts::PI / 3.0)
            && sun.get_theta() < (std::f32::consts::PI * (5.0 / 3.0))
        {
            self.light.strength = 300.0
                * ((sun.get_theta() / 2.0).sin() - sun.get_theta().cos())
                    .powf(1.5)
                    .min(1.0)
                * !self.gathered as u32 as f32;
        } else {
            self.light.strength = 0.0
        }

        self.light.position += self.velocity * FIREFLY_SPEED * time.delta_seconds();

        let mut rng = rand::thread_rng();
        let random_vec3 = Vec3::new(
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
            rng.gen_range(-1.0..1.0),
        )
        .normalize_or_zero();
        let origin_direction = (self.origin - self.light.position).normalize_or_zero();
        let origin_bias = ((self.origin - self.light.position).length() - 100.0) / 100.0;
        self.velocity = (self.velocity + random_vec3 * 0.1 + origin_direction * origin_bias)
            .normalize_or_zero();

        self.light.position.y = self.light.position.y.clamp(5.0, 15.0);
        self.render.transform.translation = self.light.position + Vec3::new(0.0, 5.0, 0.0);

        let v = Vec3::new(self.velocity.x, 0.0, self.velocity.z).normalize();
        let rotation = Quat::from_rotation_arc(Vec3::new(0.0, 0.0, 1.0), v);
        self.render.transform.rotation = rotation;
    }
}

impl Emissive for Firefly {
    fn get_lights(&self, _: &Data) -> Vec<Light> {
        vec![self.light]
    }
}

impl Renderable for Firefly {
    fn get_objects(&self) -> Vec<RenderObject> {
        if self.light.strength != 0.0 && !self.gathered {
            vec![self.render.clone()]
        } else {
            vec![]
        }
    }
}

impl Named for Firefly {
    fn get_name(&self) -> String {
        "Sunset Firefly".to_owned()
    }
}

impl Positioned for Firefly {
    fn get_position(&self) -> Vec3 {
        self.light.position
    }
}

impl Interactable for Firefly {
    fn interact(&mut self, data: &mut crate::data::Data) {
        data.inventory.add(ItemStack {
            item: Item::Fireglow,
            amount: 1,
        });
        self.gathered = true;
    }

    fn active(&self) -> bool {
        !self.gathered && self.light.strength > 0.0
    }
}

```


=== entities::player.rs

```pretty-rs
// player.rs
use std::{
    f32::consts::PI,
    sync::{Arc, Mutex},
};

use ash::vk;
use assets::{ModelRegistry, Transform};
use common::{
    item::{Item, ItemStack},
    net,
};
use glam::{Vec2, Vec3};
use winit::event::VirtualKeyCode;

use crate::{
    camera::Camera,
    data::Data,
    input::{Keyboard, Mouse},
    renderer::Renderer,
    socket::Socket,
    systems::{
        render::{Emissive, Light, RenderObject, Renderable},
        Positioned, Systems,
    },
    time::Time,
};

const PLAYER_SPEED: f32 = 100.0;
const JUMP_HEIGHT: f32 = 100.0;
const JUMP_SPEED: f32 = 4.0;
const DASH_DISTANCE: f32 = 100.0;

#[derive(Clone)]
pub struct Player {
    pub player: RenderObject,
    jump_t: f32,
    pub light: Light,
}

impl Player {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
        transform: Transform,
    ) -> Result<Arc<Mutex<Self>>, vk::Result> {
        let player = RenderObject {
            model: model_registry.load("player.glb"),
            transform,
        };

        let player = Arc::new(Mutex::new(Self {
            player,
            jump_t: 0.0,
            light: Light::new(Vec3::ZERO, 5000.0, Vec3::new(1.0, 1.0, 1.0)),
        }));

        systems.render.add(player.clone());
        systems.render.add_light(player.clone());

        Ok(player)
    }

    pub fn frame_finished(
        &mut self,
        keyboard: &Keyboard,
        mouse: &Mouse,
        camera: &Camera,
        time: &Time,
        viewport: Vec2,
        socket: &Socket,
    ) {
        let old_translation = self.player.transform.translation.clone();

        // Dash
        if keyboard.is_key_pressed(VirtualKeyCode::Space) && self.jump_t >= (PI / 4.0) {
            let mouse_direction = (mouse.position - (viewport / 2.0)).normalize_or_zero();
            let mouse_direction =
                camera.get_rotation() * Vec3::new(mouse_direction.x, 0.0, mouse_direction.y);
            self.player.transform.translation += mouse_direction * DASH_DISTANCE
        }

        // Jump
        if keyboard.is_key_pressed(VirtualKeyCode::Space) && self.jump_t == 0.0 {
            self.jump_t = std::f32::consts::PI - 0.0001;
        }

        self.player.transform.translation.y = self.jump_t.sin().powf(0.6) * JUMP_HEIGHT;
        self.jump_t -= time.delta_seconds() * JUMP_SPEED;
        self.jump_t = self.jump_t.max(0.0);

        // Movement
        let z = keyboard.is_key_down(VirtualKeyCode::W) as i32
            - keyboard.is_key_down(VirtualKeyCode::S) as i32;
        let x = keyboard.is_key_down(VirtualKeyCode::D) as i32
            - keyboard.is_key_down(VirtualKeyCode::A) as i32;
        if x != 0 || z != 0 {
            let delta = Vec3::new(x as f32, 0.0, z as f32).normalize()
                * PLAYER_SPEED
                * time.delta_seconds();
            self.player.transform.translation += camera.get_rotation() * delta;
        }

        self.light.position = self.player.transform.translation + Vec3::new(0.0, 15.0, 0.0);

        if old_translation != self.player.transform.translation {
            let packet = net::server::Packet::Move(net::server::Move {
                position: self.player.transform.translation.clone(),
            });
            socket.send(&packet).unwrap();
        }
    }
}

impl Emissive for Player {
    fn get_lights(&self, data: &Data) -> Vec<Light> {
        if data
            .inventory
            .get_items()
            .iter()
            .find(|stack| stack.item == Item::Lamp)
            .is_some()
        {
            vec![self.light]
        } else {
            Vec::new()
        }
    }
}

impl Renderable for Player {
    fn get_objects(&self) -> Vec<RenderObject> {
        vec![self.player.clone()]
    }
}

impl Positioned for Player {
    fn get_position(&self) -> Vec3 {
        self.player.transform.translation
    }
}

```


=== aetheria::input.rs

```pretty-rs
// input.rs
use glam::{UVec2, Vec2};
use std::collections::{HashMap, HashSet};

#[derive(Default)]
pub struct Keyboard {
    down: HashSet<winit::event::VirtualKeyCode>,
    pressed: HashSet<winit::event::VirtualKeyCode>,
}

impl Keyboard {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_key_down(&self, key: winit::event::VirtualKeyCode) -> bool {
        self.down.contains(&key)
    }

    pub fn is_key_pressed(&self, key: winit::event::VirtualKeyCode) -> bool {
        self.pressed.contains(&key)
    }

    pub fn on_event(&mut self, event: &winit::event::Event<()>) {
        if let winit::event::Event::DeviceEvent { event, .. } = event {
            if let winit::event::DeviceEvent::Key(key) = event {
                if let Some(keycode) = key.virtual_keycode {
                    match key.state {
                        winit::event::ElementState::Pressed => {
                            self.down.insert(keycode);
                            self.pressed.insert(keycode)
                        }
                        winit::event::ElementState::Released => self.down.remove(&keycode),
                    };
                }
            }
        }

        if let winit::event::Event::WindowEvent { event, .. } = event {
            if let winit::event::WindowEvent::Focused(false) = event {
                self.down.clear();
                self.pressed.clear();
            }
        }
    }

    pub fn frame_finished(&mut self) {
        self.pressed.clear();
    }
}

#[derive(Default)]
pub struct Mouse {
    pub delta: Vec2,
    pub position: Vec2,
    down: HashSet<winit::event::MouseButton>,
    pressed: HashSet<winit::event::MouseButton>,
    scale_factor: Vec2,
}

impl Mouse {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_button_down(&self, key: winit::event::MouseButton) -> bool {
        self.down.contains(&key)
    }

    pub fn is_button_pressed(&self, key: winit::event::MouseButton) -> bool {
        self.pressed.contains(&key)
    }

    pub fn on_event(&mut self, event: &winit::event::Event<()>) {
        if let winit::event::Event::WindowEvent { event, .. } = event {
            if let winit::event::WindowEvent::MouseInput { state, button, .. } = event {
                match state {
                    winit::event::ElementState::Pressed => {
                        self.down.insert(*button);
                        self.pressed.insert(*button)
                    }
                    winit::event::ElementState::Released => self.down.remove(button),
                };
            }

            if let winit::event::WindowEvent::CursorMoved { position, .. } = event {
                self.position = Vec2::new(position.x as f32, position.y as f32);
            }

            if let winit::event::WindowEvent::Resized(size) = event {
                self.scale_factor =
                    Vec2::new(size.width as f32 / 480.0, size.height as f32 / 270.0);
            }
        }

        if let winit::event::Event::DeviceEvent { event, .. } = event {
            if let winit::event::DeviceEvent::MouseMotion { delta } = event {
                self.delta.x = delta.0 as f32;
                self.delta.y = delta.1 as f32;
            }
        }
    }

    pub fn get_position(&self) -> UVec2 {
        UVec2::new(
            (self.position.x / self.scale_factor.x) as u32,
            (self.position.y / self.scale_factor.y) as u32,
        )
    }

    pub fn frame_finished(&mut self) {
        self.pressed.clear();
        self.delta = Vec2::ZERO;
    }
}

```


=== assets::build.rs

```pretty-rs
// build.rs
#![feature(let_chains)]

use image::io::Reader as ImageReader;
use std::{
    fs::{self, File},
    io::{Read, Write},
    path::PathBuf,
};

fn main() {
    // SHADERS

    let compiler = shaderc::Compiler::new().unwrap();
    let options = shaderc::CompileOptions::new().unwrap();
    let shader_source_paths: Vec<PathBuf> = fs::read_dir("shaders")
        .unwrap()
        .filter_map(|entry| {
            if let Ok(entry) = entry.as_ref() && let Some(extension) = entry.path().extension() && extension == "glsl" {
                Some(entry.path())
            } else {
                None
            }
        })
        .collect();

    for shader_source_path in &shader_source_paths {
        println!("cargo:rerun-if-changed={}", shader_source_path.display());
    }

    let shader_output_paths: Vec<PathBuf> = shader_source_paths
        .iter()
        .map(|path| {
            PathBuf::from(format!(
                "shaders/compiled/{}.spv",
                path.file_stem().unwrap().to_str().unwrap()
            ))
        })
        .collect();

    std::iter::zip(shader_source_paths, shader_output_paths).for_each(|(input, output)| {
        let mut file = File::open(&input).unwrap();
        let mut buf = Vec::new();
        file.read_to_end(&mut buf).unwrap();

        let source = String::from_utf8(buf).unwrap();

        let kind = match input
            .file_stem()
            .unwrap()
            .to_str()
            .unwrap()
            .split(".")
            .last()
            .unwrap()
        {
            "vert" => shaderc::ShaderKind::Vertex,
            "frag" => shaderc::ShaderKind::Fragment,
            "comp" => shaderc::ShaderKind::Compute,
            shader_type => panic!("Unexpected shader type: {}", shader_type),
        };

        let spirv = compiler
            .compile_into_spirv(
                &source,
                kind,
                input.file_name().unwrap().to_str().unwrap(),
                "main",
                Some(&options),
            )
            .unwrap();

        let mut output_file = File::create(output).unwrap();
        output_file.write_all(spirv.as_binary_u8()).unwrap();
    });

    // TEXTURES

    let texture_source_paths: Vec<PathBuf> = fs::read_dir("textures")
        .unwrap()
        .filter_map(|entry| {
            if let Ok(entry) = entry.as_ref() && let Some(extension) = entry.path().extension() && (extension == "png" || extension == "jpg")  {
                Some(entry.path())
            } else {
                None
            }
        })
        .collect();

    for texture_source_path in &texture_source_paths {
        println!("cargo:rerun-if-changed={}", texture_source_path.display());
    }

    let texture_output_paths: Vec<PathBuf> = texture_source_paths
        .iter()
        .map(|path| {
            PathBuf::from(format!(
                "textures/compiled/{}.qoi",
                path.file_stem().unwrap().to_str().unwrap()
            ))
        })
        .collect();

    std::iter::zip(texture_source_paths, texture_output_paths).for_each(|(input, output)| {
        let image = ImageReader::open(input).unwrap().decode().unwrap();
        let bytes = image.to_rgba8().to_vec();
        let encoded = qoi::encode_to_vec(bytes, image.width(), image.height()).unwrap();

        let mut output_file = File::create(output).unwrap();
        output_file.write_all(&encoded).unwrap();
    });
}

```


=== systems::interact.rs

```pretty-rs
// interact.rs
use glam::{IVec2, Quat, UVec2, Vec3};

use crate::{
    camera::Camera,
    components,
    data::{inventory::Inventory, Data},
    entities::Player,
    input::Keyboard,
    renderer::{RENDER_HEIGHT, RENDER_WIDTH},
    ui::{Element, Rectangle, Region, SizeConstraints},
};

use super::{Named, Positioned};

use std::{
    f32::consts::PI,
    sync::{Arc, Mutex, Weak},
};

pub struct System {
    interactables: Vec<Weak<Mutex<dyn Interactable>>>,
    player: Option<Weak<Mutex<Player>>>,
}

impl System {
    pub fn new() -> Self {
        Self {
            interactables: Vec::new(),
            player: None,
        }
    }

    pub fn add<T: Interactable + Sized + 'static>(&mut self, interactable: Arc<Mutex<T>>) {
        self.interactables.push(Arc::downgrade(
            &(interactable as Arc<Mutex<dyn Interactable>>),
        ))
    }

    pub fn set_player(&mut self, player: Arc<Mutex<Player>>) {
        self.player = Some(Arc::downgrade(&player));
    }

    pub fn frame_finished(
        &mut self,
        camera: &Camera,
        keyboard: &Keyboard,
        scene: &mut Vec<Rectangle>,
        data: &mut Data,
    ) {
        if self.player.is_none() || self.player.as_ref().unwrap().upgrade().is_none() {
            return;
        }

        let camera_delta =
            Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), 2.0 * PI - camera.actual_theta)
                * (camera.target - camera.actual_target);
        let player_position = self
            .player
            .as_ref()
            .unwrap()
            .upgrade()
            .unwrap()
            .lock()
            .unwrap()
            .get_position();
        let mut distances = self
            .interactables
            .iter()
            .enumerate()
            .filter_map(|(i, interactable)| interactable.upgrade().map(|g| (i, g)))
            .filter(|(_, interactable)| interactable.lock().unwrap().active())
            .map(|(i, interactable)| {
                (
                    i,
                    (interactable.lock().unwrap().get_position() - player_position).length(),
                )
            })
            .collect::<Vec<(usize, f32)>>();

        distances.sort_by(|(_, a), (_, b)| a.total_cmp(&b));
        let Some(closest) = distances.first() else {
            return;
        };

        if closest.1 < 50.0 {
            let interactable = self.interactables[closest.0].upgrade().unwrap();
            let mut widget =
                components::interact::Component::new(&interactable.lock().unwrap().get_name());
            let size = widget.layout(SizeConstraints {
                min: UVec2::new(0, 0),
                max: UVec2::new(RENDER_WIDTH, RENDER_HEIGHT),
            });

            let origin = IVec2::new(250, 145)
                + IVec2::new(
                    camera_delta.x as i32,
                    (camera_delta.z * 2.0_f32.powf(-0.5)) as i32,
                );
            widget.paint(
                Region {
                    origin: UVec2::new(origin.x as u32, origin.y as u32),
                    size,
                },
                scene,
            );

            if keyboard.is_key_pressed(winit::event::VirtualKeyCode::F) {
                interactable.lock().unwrap().interact(data);
            }
        }
    }
}

pub trait Interactable: Named + Positioned {
    fn interact(&mut self, data: &mut Data);
    fn active(&self) -> bool;
}

```


=== scenes::root.rs

```pretty-rs
// root.rs
use std::{
    ops::Deref,
    sync::{Arc, Mutex},
};

use ash::vk;
use assets::{ModelRegistry, Transform};
use glam::{Quat, Vec2, Vec3};

use crate::{
    camera::Camera,
    entities::{CraftingBench, Furnace, Grass, Player, Sun},
    input::{Keyboard, Mouse},
    renderer::Renderer,
    socket::Socket,
    systems::{render::Light, Systems},
    time::Time,
};

use super::{Fireflies, Ores, Trees};

pub struct RootScene {
    pub player: Arc<Mutex<Player>>,
    pub sun: Arc<Mutex<Sun>>,
    pub grass: Arc<Mutex<Grass>>,
    pub trees: Trees,
    pub fireflies: Fireflies,
    pub furnace: Arc<Mutex<Furnace>>,
    pub crafting_bench: Arc<Mutex<CraftingBench>>,
    pub ores: Ores,
}

impl RootScene {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
    ) -> Result<Self, vk::Result> {
        let player = {
            let transform = Transform {
                translation: Vec3::new(0.0, 10.0, 0.0),
                rotation: Quat::IDENTITY,
                scale: Vec3::ONE,
            };
            Player::new(renderer, systems, model_registry, transform).unwrap()
        };
        let sun = Sun::new(
            systems,
            Vec3::new(0.0, 1000000.0, 0.0),
            Vec3::new(0.8, 1.0, 0.5),
        );
        let grass = Grass::new(renderer, systems, model_registry, Transform::IDENTITY).unwrap();

        let trees = Trees::new(renderer, systems, model_registry)?;
        let fireflies = Fireflies::new(renderer, systems, model_registry)?;

        let furnace = Furnace::new(
            renderer,
            systems,
            model_registry,
            Transform {
                translation: Vec3::new(100.0, 0.0, 100.0),
                scale: Vec3::new(0.2, 0.2, 0.2),
                ..Default::default()
            },
        )?;

        let ores = Ores::new(renderer, systems, model_registry)?;

        let crafting_bench = CraftingBench::new(
            renderer,
            systems,
            model_registry,
            Transform {
                translation: Vec3::new(100.0, 0.0, 30.0),
                rotation: Quat::IDENTITY,
                scale: Vec3::new(0.1, 0.1, 0.1),
            },
        )?;
        Ok(Self {
            player,
            sun,
            grass,
            trees,
            fireflies,
            furnace,
            crafting_bench,
            ores,
        })
    }

    pub fn frame_finished(
        &mut self,
        keyboard: &Keyboard,
        mouse: &Mouse,
        camera: &Camera,
        time: &Time,
        viewport: Vec2,
        socket: &Socket,
    ) {
        self.player
            .lock()
            .unwrap()
            .frame_finished(keyboard, mouse, camera, time, viewport, socket);
        self.sun.lock().unwrap().frame_finished(time);
        self.fireflies.iter_mut().for_each(|firefly| {
            firefly
                .lock()
                .unwrap()
                .frame_finished(&self.sun.lock().unwrap(), time)
        });
    }
}

```


=== vulkan::context.rs

```pretty-rs
// context.rs
use super::{allocator::Allocator, command, Device, Instance, Surface, Swapchain};
use ash::{vk, Entry};
use std::sync::{Arc, Mutex};

pub struct Context {
    pub instance: Instance,
    pub surface: Surface,
    pub device: Arc<Device>,
    pub swapchain: Swapchain,
    pub command_pool: command::Pool,

    pub image_available: vk::Semaphore,

    pub allocator: Arc<Mutex<Allocator>>,
}

impl Context {
    pub fn new(window: &winit::window::Window) -> Self {
        let entry = Entry::linked();
        let instance = Instance::new(&entry).expect("Vulkan instance creation failed");
        let surface = Surface::new(&instance, window).expect("Vulkan surface creation failed");
        let device = unsafe {
            Arc::new(Device::new(&instance, &surface).expect("Vulkan device creation failed"))
        };

        let swapchain = Swapchain::new(&instance, &surface, &device, window)
            .expect("Vulkan swapchain creation failed");

        let command_pool = command::Pool::new(device.clone()).unwrap();

        let semaphore_info = vk::SemaphoreCreateInfo::builder();
        let image_available = unsafe { device.create_semaphore(&semaphore_info, None).unwrap() };

        let allocator = Allocator::new(&instance, device.clone()).unwrap();

        Self {
            instance,
            surface,
            device,
            swapchain,
            command_pool,
            image_available,
            allocator: Arc::new(Mutex::new(allocator)),
        }
    }

    pub unsafe fn start_frame(&mut self, in_flight: vk::Fence) -> Result<u32, vk::Result> {
        unsafe {
            let image_index = self
                .device
                .extensions
                .swapchain
                .as_ref()
                .unwrap()
                .acquire_next_image(
                    self.swapchain.swapchain,
                    u64::MAX,
                    self.image_available,
                    vk::Fence::null(),
                )?
                .0;

            self.device.reset_fences(&[in_flight]).unwrap();
            self.allocator.lock().unwrap().flush_frees();

            Ok(image_index)
        }
    }

    pub unsafe fn end_frame(
        &self,
        image_index: u32,
        render_finished: vk::Semaphore,
    ) -> Result<(), vk::Result> {
        unsafe {
            let signal_semaphores = &[render_finished];
            let swapchains = &[self.swapchain.swapchain];
            let image_indices = &[image_index];
            let present_info = vk::PresentInfoKHR::builder()
                .wait_semaphores(signal_semaphores)
                .swapchains(swapchains)
                .image_indices(image_indices);

            self.device
                .extensions
                .swapchain
                .as_ref()
                .unwrap()
                .queue_present(self.device.queues.present.queue, &present_info)?;
        }

        Ok(())
    }
}

```


=== entities::crafting_bench.rs

```pretty-rs
// crafting_bench.rs
use crate::{
    data::{Data, Recipe},
    renderer::Renderer,
    systems::{
        interact::Interactable,
        render::{RenderObject, Renderable},
        Named, Positioned, Systems,
    },
};
use ash::vk;
use assets::{ModelRegistry, Transform};
use common::item::{Item, ItemStack};
use glam::Vec3;
use std::sync::{Arc, Mutex};

pub struct CraftingBench {
    render: RenderObject,
}

impl CraftingBench {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
        transform: Transform,
    ) -> Result<Arc<Mutex<Self>>, vk::Result> {
        let render = RenderObject {
            model: model_registry.load("crafting_bench.glb"),
            transform,
        };

        let bench = Arc::new(Mutex::new(Self { render }));

        systems.render.add(bench.clone());
        systems.interact.add(bench.clone());

        Ok(bench)
    }
}

impl Renderable for CraftingBench {
    fn get_objects(&self) -> Vec<RenderObject> {
        vec![self.render.clone()]
    }
}

impl Named for CraftingBench {
    fn get_name(&self) -> String {
        "Crafting Bench".to_owned()
    }
}

impl Positioned for CraftingBench {
    fn get_position(&self) -> Vec3 {
        self.render.transform.translation
    }
}

impl Interactable for CraftingBench {
    fn active(&self) -> bool {
        true
    }

    fn interact(&mut self, data: &mut Data) {
        data.recipe_selections = Some(vec![
            Recipe {
                ingredients: vec![
                    ItemStack {
                        item: Item::Wood,
                        amount: 3,
                    },
                    ItemStack {
                        item: Item::Fireglow,
                        amount: 2,
                    },
                ],
                outputs: vec![ItemStack {
                    item: Item::Lamp,
                    amount: 1,
                }],
            },
            Recipe {
                ingredients: vec![
                    ItemStack {
                        item: Item::Wood,
                        amount: 2,
                    },
                    ItemStack {
                        item: Item::CopperIngot,
                        amount: 2,
                    },
                ],
                outputs: vec![ItemStack {
                    item: Item::CopperSword,
                    amount: 1,
                }],
            },
        ])
    }
}

```


=== data::inventory.rs

```pretty-rs
// inventory.rs
use common::{
    item::{Item, ItemStack},
    net,
};
use std::sync::Arc;
use tracing::warn;

use crate::socket::Socket;

#[derive(Clone)]
pub struct Inventory {
    inventory: Vec<ItemStack>,
    socket: Arc<Socket>,
}

impl Inventory {
    pub fn new(socket: Arc<Socket>) -> Self {
        Self {
            inventory: Vec::new(),
            socket,
        }
    }

    fn update(&self, item: Item) {
        let Some(stack) = self.inventory.iter().find(|s| s.item == item) else {
            warn!("Tried to update stack {:?} that doesn't exist", item);
            return;
        };

        let packet = net::server::Packet::ModifyInventory(net::server::ModifyInventory {
            stack: stack.clone(),
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

    pub fn remove(&mut self, stack: ItemStack) {
        if let Some((i, existing)) = self
            .inventory
            .iter_mut()
            .enumerate()
            .find(|(_, s)| s.item == stack.item)
        {
            if existing.amount < stack.amount {
                warn!(
                    "Removing {} from inventory would give negative items",
                    stack
                );
                return;
            }

            existing.amount -= stack.amount;

            if existing.amount == 0 {
                self.inventory.remove(i);
            }
        } else {
            warn!("Tried to remove {} but no such stack existed", stack);
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


=== entities::furnace.rs

```pretty-rs
// furnace.rs
use crate::{
    data::{Data, Recipe},
    renderer::Renderer,
    systems::{
        interact::Interactable,
        render::{Emissive, Light, RenderObject, Renderable},
        Named, Positioned, Systems,
    },
};
use ash::vk;
use assets::{ModelRegistry, Transform};
use common::item::{Item, ItemStack};
use glam::Vec3;
use std::sync::{Arc, Mutex};

pub struct Furnace {
    render: RenderObject,
    light: Light,
}

impl Furnace {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
        transform: Transform,
    ) -> Result<Arc<Mutex<Self>>, vk::Result> {
        let render = RenderObject {
            model: model_registry.load("furnace.glb"),
            transform: transform.clone(),
        };

        let light = Light::new(
            transform.translation + Vec3::new(0.0, 20.0, -10.0),
            4000.0,
            Vec3::new(0.976, 0.451, 0.086),
        );

        let furnace = Arc::new(Mutex::new(Self { render, light }));
        systems.render.add(furnace.clone());
        systems.render.add_light(furnace.clone());
        systems.interact.add(furnace.clone());

        Ok(furnace)
    }
}

impl Renderable for Furnace {
    fn get_objects(&self) -> Vec<RenderObject> {
        vec![self.render.clone()]
    }
}

impl Named for Furnace {
    fn get_name(&self) -> String {
        "Furnace".to_owned()
    }
}

impl Positioned for Furnace {
    fn get_position(&self) -> Vec3 {
        self.render.transform.translation
    }
}

impl Interactable for Furnace {
    fn interact(&mut self, data: &mut Data) {
        data.current_recipe = Some(Recipe {
            ingredients: vec![ItemStack {
                item: Item::CopperOre,
                amount: 3,
            }],
            outputs: vec![ItemStack {
                item: Item::CopperIngot,
                amount: 1,
            }],
        })
    }

    fn active(&self) -> bool {
        true
    }
}

impl Emissive for Furnace {
    fn get_lights(&self, _: &Data) -> Vec<Light> {
        vec![self.light]
    }
}

```


=== common::net.rs

```pretty-rs
// net.rs
mod common {
    use crate::item::ItemStack;
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct ModifyInventory {
        pub stack: ItemStack,
    }
}

pub mod server {
    pub use super::common::ModifyInventory;
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct Login {
        pub username: String,
        pub password: String,
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct Move {
        pub position: glam::Vec3,
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub struct Signup {
        pub username: String,
        pub password: String,
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub enum Packet {
        Login(Login),
        Move(Move),
        Heartbeat,
        Disconnect,
        ModifyInventory(ModifyInventory),
        Signup(Signup),
    }
}

pub mod client {
    pub use super::common::ModifyInventory;
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
    pub struct DisplayError {
        pub message: String,
        pub fatal: bool,
    }

    #[derive(Serialize, Deserialize, Debug, Clone)]
    pub enum Packet {
        SpawnPlayer(SpawnPlayer),
        DespawnPlayer(DespawnPlayer),
        Move(Move),
        NotifyDisconnection(NotifyDisconnection),
        ModifyInventory(ModifyInventory),
        DisplayError(DisplayError),
    }
}

```


=== aetheria::camera.rs

```pretty-rs
// camera.rs
use std::f32::EPSILON;

use ash::vk;
use bytemuck::{cast_slice, cast_slice_mut};
use glam::{Mat4, Quat, Vec3};
use vulkan::Buffer;

use crate::renderer::Renderer;

pub struct Camera {
    pub target: Vec3,
    pub actual_target: Vec3,
    pub theta: f32,
    pub actual_theta: f32,

    pub width: f32,
    pub height: f32,

    pub buffer: Buffer,
}

impl Camera {
    const DAMPING: f32 = 0.2;

    pub fn new(width: f32, height: f32, renderer: &Renderer) -> Result<Self, vk::Result> {
        //let theta = -45.01_f32.to_radians();
        let theta = 0.0;
        let target = Vec3::new(0.0, 0.0, 0.0);

        let camera = Self {
            theta,
            actual_theta: theta,
            target,
            actual_target: target,
            width,
            height,
            buffer: Buffer::new(&renderer, [0_u8; 32], vk::BufferUsageFlags::UNIFORM_BUFFER)?,
        };

        Ok(camera)
    }

    fn pad_vec3(data: Vec3) -> [f32; 4] {
        [data.x, data.y, data.z, 0.0]
    }

    pub fn update_buffer(&mut self) {
        let mut eye = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), self.actual_theta)
            * Vec3::new(0.0, 500.0 * 2.0_f32.powf(-0.5), -500.0);
        eye += self.actual_target;

        let vp = [Self::pad_vec3(eye), Self::pad_vec3(self.actual_target)]
            .iter()
            .flatten()
            .copied()
            .collect::<Vec<f32>>();
        let vp = cast_slice::<f32, u8>(&vp);
        self.buffer.upload(vp);
    }

    pub fn frame_finished(&mut self) {
        if (self.actual_theta - self.theta).abs() > EPSILON {
            self.actual_theta += (self.theta - self.actual_theta) * Self::DAMPING;
        }

        if (self.actual_target - self.target).length() > EPSILON {
            self.actual_target += (self.target - self.actual_target) * Self::DAMPING;
        }

        self.update_buffer();
    }

    pub fn get_rotation(&self) -> Quat {
        Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), self.theta)
    }
}

```


=== entities::copper_ore.rs

```pretty-rs
// copper_ore.rs
use crate::{
    data::Data,
    renderer::Renderer,
    systems::{
        interact::Interactable,
        render::{RenderObject, Renderable},
        Named, Positioned, Systems,
    },
};
use ash::vk;
use assets::{ModelRegistry, Transform};
use common::item::{Item, ItemStack};
use glam::Vec3;
use std::sync::{Arc, Mutex};

pub struct CopperOre {
    render: RenderObject,
    gathered: bool,
}

impl CopperOre {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
        transform: Transform,
    ) -> Result<Arc<Mutex<Self>>, vk::Result> {
        let render = RenderObject {
            model: model_registry.load("copper_ore.glb"),
            transform,
        };

        let ore = Arc::new(Mutex::new(Self {
            render,
            gathered: false,
        }));

        systems.render.add(ore.clone());
        systems.interact.add(ore.clone());

        Ok(ore)
    }
}

impl Renderable for CopperOre {
    fn get_objects(&self) -> Vec<RenderObject> {
        if self.gathered {
            Vec::new()
        } else {
            vec![self.render.clone()]
        }
    }
}

impl Named for CopperOre {
    fn get_name(&self) -> String {
        "Copper Ore".to_owned()
    }
}

impl Positioned for CopperOre {
    fn get_position(&self) -> Vec3 {
        self.render.transform.translation.clone()
    }
}

impl Interactable for CopperOre {
    fn active(&self) -> bool {
        !self.gathered
    }

    fn interact(&mut self, data: &mut Data) {
        data.inventory.add(ItemStack {
            item: Item::CopperOre,
            amount: 1,
        });
        self.gathered = true;
    }
}

```


=== entities::tree.rs

```pretty-rs
// tree.rs
use std::sync::{Arc, Mutex};

use ash::vk;
use assets::{ModelRegistry, Transform};
use glam::Vec3;

use crate::{
    data::{inventory::Inventory, Data},
    renderer::Renderer,
    systems::{
        interact::Interactable,
        render::{RenderObject, Renderable},
        Named, Positioned, Systems,
    },
};
use common::item::{Item, ItemStack};

pub struct Tree {
    pub tree: RenderObject,
    gathered: bool,
}

impl Tree {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
        transform: Transform,
    ) -> Result<Arc<Mutex<Tree>>, vk::Result> {
        let tree = RenderObject {
            model: model_registry.load("tree.glb"),
            transform,
        };

        let tree = Arc::new(Mutex::new(Self {
            tree,
            gathered: false,
        }));

        systems.render.add(tree.clone());
        systems.interact.add(tree.clone());

        Ok(tree)
    }
}

impl Renderable for Tree {
    fn get_objects(&self) -> Vec<RenderObject> {
        if !self.gathered {
            vec![self.tree.clone()]
        } else {
            Vec::new()
        }
    }
}

impl Named for Tree {
    fn get_name(&self) -> String {
        "Tree".to_owned()
    }
}

impl Positioned for Tree {
    fn get_position(&self) -> Vec3 {
        self.tree.transform.translation
    }
}

impl Interactable for Tree {
    fn interact(&mut self, data: &mut Data) {
        data.inventory.add(ItemStack {
            item: Item::Wood,
            amount: 1,
        });
        self.gathered = true;
    }

    fn active(&self) -> bool {
        !self.gathered
    }
}

```


=== vulkan::buffer.rs

```pretty-rs
// buffer.rs
use super::{
    allocator::{Allocation, Allocator},
    Context,
};
use ash::vk::{self, MemoryPropertyFlags};
use std::sync::{Arc, Mutex};
use std::{
    ops::{Deref, Drop},
    result::Result,
};

pub struct Buffer {
    pub(crate) buffer: vk::Buffer,
    pub(crate) allocation: Allocation,
    pub size: usize,
    allocator: Arc<Mutex<Allocator>>,
}

impl Buffer {
    pub fn new<T: Into<Vec<u8>>>(
        ctx: &Context,
        data: T,
        usage: vk::BufferUsageFlags,
    ) -> Result<Self, vk::Result> {
        let bytes: Vec<u8> = data.into();

        let create_info = vk::BufferCreateInfo::builder()
            .size(bytes.len() as u64)
            .usage(usage);

        let (buffer, allocation) = ctx.allocator.lock().unwrap().create_buffer(
            &create_info,
            MemoryPropertyFlags::DEVICE_LOCAL
                | MemoryPropertyFlags::HOST_VISIBLE
                | MemoryPropertyFlags::HOST_COHERENT,
        )?;

        ctx.allocator.lock().unwrap().write(&allocation, &bytes)?;

        Ok(Self {
            buffer,
            allocation,
            size: bytes.len(),
            allocator: ctx.allocator.clone(),
        })
    }

    pub fn upload(&self, bytes: &[u8]) {
        self.allocator
            .lock()
            .unwrap()
            .write(&self.allocation, bytes)
            .expect("Failed to write to buffer");
    }
}

impl Deref for Buffer {
    type Target = vk::Buffer;

    fn deref(&self) -> &Self::Target {
        &self.buffer
    }
}

impl Drop for Buffer {
    fn drop(&mut self) {
        self.allocator.lock().unwrap().free(&self.allocation);
    }
}

```


=== vulkan::surface.rs

```pretty-rs
// surface.rs
use super::Instance;
use ash::vk;
use std::{ffi::c_void, ops::Deref, result::Result};
use winit::window::Window;

#[cfg(target_os = "linux")]
use winit::platform::x11::WindowExtX11;

#[cfg(target_os = "windows")]
use winit::platform::windows::WindowExtWindows;

pub struct Surface {
    pub(crate) surface: vk::SurfaceKHR,
}

impl Surface {
    #[cfg(target_os = "linux")]
    pub fn new(instance: &Instance, window: &Window) -> Result<Self, vk::Result> {
        let create_info = vk::XlibSurfaceCreateInfoKHR::builder()
            .dpy(window.xlib_display().unwrap().cast::<*const c_void>())
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

    #[cfg(target_os = "windows")]
    pub fn new(instance: &Instance, window: &Window) -> Result<Self, vk::Result> {
        let create_info = vk::Win32SurfaceCreateInfoKHR::builder()
            .hinstance(window.hinstance() as *const c_void)
            .hwnd(window.hwnd() as *const c_void);

        let surface = unsafe {
            instance
                .extensions
                .win32_surface
                .as_ref()
                .unwrap()
                .create_win32_surface(&create_info, None)?
        };

        Ok(Self { surface })
    }
}

impl Deref for Surface {
    type Target = vk::SurfaceKHR;

    fn deref(&self) -> &Self::Target {
        &self.surface
    }
}

```


=== entities::sun.rs

```pretty-rs
// sun.rs
use std::{
    f32::consts::PI,
    ops::Deref,
    sync::{Arc, Mutex},
    time::SystemTime,
};

use glam::{Quat, Vec3};

use crate::{
    data::Data,
    systems::{
        render::{Emissive, Light},
        Systems,
    },
    time::Time,
};

pub struct Sun {
    noon_pos: Vec3,
    pub light: Light,
    theta: f32,
}

impl Sun {
    pub fn new(systems: &mut Systems, noon_pos: Vec3, color: Vec3) -> Arc<Mutex<Self>> {
        let seconds = SystemTime::UNIX_EPOCH.elapsed().unwrap().as_secs();
        let mut sun = Self {
            noon_pos,
            light: Light::new(noon_pos, 0.0, color),
            theta: (seconds % 120) as f32 * (PI / 60.0),
        };
        sun.update_theta(sun.theta);

        let sun = Arc::new(Mutex::new(sun));

        systems.render.add_light(sun.clone());

        sun
    }

    pub fn update_theta(&mut self, theta: f32) {
        self.theta = theta % (std::f32::consts::PI * 2.0);
        self.light.position =
            Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), self.theta) * self.noon_pos;
        self.light.color = Vec3::new(1.0, 1.0, 1.0);
        self.light.strength =
            self.light.position.length().powf(2.0) * 1.5 * self.theta.cos().powf(0.13).max(0.0);
        self.light.strength = self.light.strength.max(0.0);
    }

    pub fn frame_finished(&mut self, time: &Time) {
        self.update_theta(self.theta + (time.delta_seconds() * (PI / 60.0)));
    }

    pub fn get_theta(&self) -> f32 {
        self.theta
    }
}

impl Emissive for Sun {
    fn get_lights(&self, _: &Data) -> Vec<Light> {
        vec![self.light]
    }
}

```


=== components::recipe_selector.rs

```pretty-rs
// recipe_selector.rs
use super::components::{Button, Container, HAlign, Handler, Padding, VList};
use crate::{
    data::{Data, Recipe},
    input::Mouse,
    ui,
};
use std::sync::{Arc, Mutex};

pub type Component<'a> = Container<Padding<VList<Button<'a, RecipeSelectorHandler<'a>>>>>;

pub struct RecipeSelectorHandler<'a> {
    recipe: Recipe,
    data: Arc<Mutex<&'a mut Data>>,
}

impl Handler for RecipeSelectorHandler<'_> {
    fn handle(&mut self) {
        self.data.lock().unwrap().current_recipe = Some(self.recipe.clone());
        self.data.lock().unwrap().recipe_selections = None;
    }
}

impl<'a> Component<'a> {
    pub fn new(data: &'a mut Data, mouse: &'a Mouse) -> Option<Self> {
        let recipes = data.recipe_selections.as_ref()?.clone();
        let data_mutex = Arc::new(Mutex::new(data));
        let buttons = recipes
            .iter()
            .map(|recipe| {
                let handler = RecipeSelectorHandler {
                    recipe: recipe.clone(),
                    data: data_mutex.clone(),
                };
                Button::new(mouse, &format!("{}", recipe.outputs[0]), handler)
            })
            .collect();

        Some(Self {
            child: Padding::new_uniform(
                VList {
                    children: buttons,
                    separation: 2,
                    align: HAlign::Left,
                },
                2,
            ),
            color: ui::color::get_background(),
            border_radius: 1,
            border_color: ui::color::get_highlight(),
        })
    }
}

```


=== scenes::ores.rs

```pretty-rs
// ores.rs
use std::{
    f32::consts::PI,
    ops::{Deref, DerefMut},
    sync::{Arc, Mutex},
};

use crate::{entities::CopperOre, renderer::Renderer, systems::Systems};
use ash::vk;
use assets::{ModelRegistry, Transform};
use glam::{Quat, Vec3};
use rand::Rng;

const NUM_ORES: u32 = 10;

pub struct Ores {
    trees: Vec<Arc<Mutex<CopperOre>>>,
}

impl Ores {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
    ) -> Result<Self, vk::Result> {
        let mut trees = Vec::new();

        let mut rng = rand::thread_rng();

        for _ in 0..NUM_ORES {
            let translation = Vec3::new(
                rng.gen_range(-400.0..400.0),
                0.0,
                rng.gen_range(-400.0..400.0),
            );
            let rotation = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), rng.gen_range(-PI..PI));
            let transform = Transform {
                translation,
                rotation,
                scale: Vec3::new(0.1, 0.1, 0.1),
            };
            trees.push(CopperOre::new(renderer, systems, model_registry, transform).unwrap());
        }

        Ok(Self { trees })
    }
}

impl Deref for Ores {
    type Target = Vec<Arc<Mutex<CopperOre>>>;

    fn deref(&self) -> &Self::Target {
        &self.trees
    }
}

impl DerefMut for Ores {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.trees
    }
}

```


=== scenes::trees.rs

```pretty-rs
// trees.rs
use std::{
    f32::consts::PI,
    ops::{Deref, DerefMut},
    sync::{Arc, Mutex},
};

use ash::vk;
use assets::{ModelRegistry, Transform};
use glam::{Quat, Vec3};
use rand::Rng;

use crate::{entities::Tree, renderer::Renderer, systems::Systems};

const NUM_TREES: u32 = 10;

pub struct Trees {
    trees: Vec<Arc<Mutex<Tree>>>,
}

impl Trees {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
    ) -> Result<Self, vk::Result> {
        let mut trees = Vec::new();

        let mut rng = rand::thread_rng();

        for _ in 0..NUM_TREES {
            let translation = Vec3::new(
                rng.gen_range(-400.0..400.0),
                0.0,
                rng.gen_range(-400.0..400.0),
            );
            let rotation = Quat::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), rng.gen_range(-PI..PI));
            let transform = Transform {
                translation,
                rotation,
                scale: Vec3::new(0.1, 0.1, 0.1),
            };
            trees.push(Tree::new(renderer, systems, model_registry, transform).unwrap());
        }

        Ok(Self { trees })
    }
}

impl Deref for Trees {
    type Target = Vec<Arc<Mutex<Tree>>>;

    fn deref(&self) -> &Self::Target {
        &self.trees
    }
}

impl DerefMut for Trees {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.trees
    }
}

```


=== vulkan::compute.rs

```pretty-rs
// compute.rs
use ash::vk::{self, DescriptorSetLayout};

use crate::{Device, SetLayout, Shader};
use std::{ops::Deref, sync::Arc};

#[derive(Clone)]
pub struct Pipeline {
    shader: Arc<Shader>,
    pub(crate) layout: vk::PipelineLayout,
    pipeline: vk::Pipeline,
}

impl Pipeline {
    pub fn new(
        device: &Device,
        shader: Arc<Shader>,
        layouts: &[SetLayout],
    ) -> Result<Self, vk::Result> {
        let stage = shader.get_stage();
        let descriptors = layouts
            .iter()
            .map(|layout| layout.layout)
            .collect::<Vec<DescriptorSetLayout>>();
        println!("Descriptors: {}", descriptors.len());
        let layout_info = vk::PipelineLayoutCreateInfo::builder().set_layouts(&descriptors);
        let layout = unsafe { device.create_pipeline_layout(&layout_info, None)? };

        let pipeline_info = vk::ComputePipelineCreateInfo::builder()
            .stage(*stage)
            .layout(layout);
        let pipeline = unsafe {
            device
                .create_compute_pipelines(vk::PipelineCache::null(), &[*pipeline_info], None)
                .expect("Failed to create compute pipeline")[0]
        };
        Ok(Self {
            shader,
            layout,
            pipeline,
        })
    }
}

impl Deref for Pipeline {
    type Target = vk::Pipeline;

    fn deref(&self) -> &Self::Target {
        &self.pipeline
    }
}

```


=== scenes::fireflies.rs

```pretty-rs
// fireflies.rs
use std::{
    ops::{Deref, DerefMut},
    sync::{Arc, Mutex},
};

use ash::vk;
use assets::{ModelRegistry, Transform};
use glam::Vec3;
use rand::Rng;

use crate::{entities::Firefly, renderer::Renderer, systems::Systems};

const NUM_FIREFLIES: u32 = 10;

pub struct Fireflies {
    fireflies: Vec<Arc<Mutex<Firefly>>>,
}

impl Fireflies {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
    ) -> Result<Self, vk::Result> {
        let mut fireflies = Vec::new();

        let mut rng = rand::thread_rng();

        for _ in 0..NUM_FIREFLIES {
            let position = Vec3::new(
                rng.gen_range(-400.0..400.0),
                50.0,
                rng.gen_range(-400.0..400.0),
            );
            fireflies.push(
                Firefly::new(
                    renderer,
                    systems,
                    model_registry,
                    position,
                    Vec3::new(1.0, 1.0, 1.0),
                )
                .unwrap(),
            );
        }

        Ok(Self { fireflies })
    }
}

impl Deref for Fireflies {
    type Target = Vec<Arc<Mutex<Firefly>>>;

    fn deref(&self) -> &Self::Target {
        &self.fireflies
    }
}

impl DerefMut for Fireflies {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.fireflies
    }
}

```


=== aetheria::time.rs

```pretty-rs
// time.rs
use ash::vk;
use bytemuck::cast_slice;
use std::time::Instant;
use tracing::info;
use vulkan::Buffer;

use crate::renderer::Renderer;

pub struct Time {
    last_frame: Instant,
    current_frame: Instant,
    pub time: f32,
    pub buffer: Buffer,
}

impl Time {
    pub fn new(renderer: &Renderer) -> Result<Self, vk::Result> {
        info!("Starting frame timer");
        let time = Self {
            last_frame: Instant::now(),
            current_frame: Instant::now(),
            time: 0.0,
            buffer: Buffer::new(renderer, [0_u8; 8], vk::BufferUsageFlags::UNIFORM_BUFFER)?,
        };
        Ok(time)
    }

    pub fn delta_seconds(&self) -> f32 {
        (self.current_frame - self.last_frame).as_secs_f32()
    }

    fn update_buffer(&mut self) {
        let delta = self.delta_seconds();
        let data = &[self.time, delta];
        let data = cast_slice::<f32, u8>(data);
        self.buffer.upload(data);
    }

    pub fn frame_finished(&mut self) {
        let delta = self.delta_seconds();
        self.time += delta;

        println!("FPS: {}", 1.0 / self.delta_seconds());

        self.last_frame = self.current_frame;
        self.current_frame = Instant::now();
        self.update_buffer();
    }
}

```


=== components::interact.rs

```pretty-rs
// interact.rs
use crate::ui::{self, Element};

use super::components::*;
use glam::Vec4;

pub type Component = Container<Padding<HPair<Container<Padding<Text>>, Text>>>;

impl Component {
    pub fn new(name: &str) -> Self {
        let f = Text {
            color: ui::color::get_highlight(),
            content: "F".to_owned(),
        };
        let padded_f = Padding {
            child: f,
            top: 1,
            bottom: 1,
            left: 1,
            right: 0,
        };
        let left = Container {
            child: padded_f,
            color: ui::color::get_background(),
            border_color: ui::color::get_highlight(),
            border_radius: 1,
        };
        let right = Text {
            color: ui::color::get_highlight(),
            content: name.to_owned(),
        };
        let hpair = HPair::new(left, right, VAlign::Center, 2);
        let padding = Padding {
            child: hpair,
            top: 2,
            bottom: 2,
            left: 2,
            right: 2,
        };
        Container {
            child: padding,
            border_radius: 1,
            border_color: ui::color::get_highlight(),
            color: ui::color::get_background(),
        }
        .into()
    }
}

```


=== vulkan::lib.rs

```pretty-rs
// lib.rs
#![feature(once_cell_try)]

pub mod instance;
pub use instance::Instance;

pub mod buffer;
pub use buffer::Buffer;

pub mod command;
pub use command::DrawOptions;

pub mod context;
pub use context::Context;

pub mod descriptor;
pub use descriptor::*;

pub mod device;
pub use device::Device;

pub mod image;
pub use image::{Image, Texture};

pub mod graphics;
pub use graphics::{Pipeline, Shader, Shaders, VertexInputBuilder};

pub mod renderpass;
pub use renderpass::Renderpass;

pub mod surface;
pub use surface::Surface;

pub mod swapchain;
pub use swapchain::Swapchain;

pub mod compute;

pub mod allocator;

use cstr::cstr;
use std::{clone::Clone, cmp::Eq, collections::HashSet, ffi::CStr, hash::Hash};

#[cfg(debug_assertions)]
fn get_wanted_layers() -> Vec<&'static CStr> {
    vec![cstr!("VK_LAYER_KHRONOS_validation")]
}

#[cfg(not(debug_assertions))]
fn get_wanted_layers() -> Vec<&'static CStr> {
    vec![]
}

fn intersection<T: Hash + Clone + Eq>(a: &[T], b: &[T]) -> Vec<T> {
    let a_unique: HashSet<T> = a.iter().cloned().collect();
    let b_unique: HashSet<T> = b.iter().cloned().collect();
    a_unique.intersection(&b_unique).cloned().collect()
}

```


=== common::item.rs

```pretty-rs
// item.rs
use num_derive::{FromPrimitive, ToPrimitive};
use serde::{Deserialize, Serialize};
use std::fmt::Display;

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, FromPrimitive, ToPrimitive)]
pub enum Item {
    Wood,
    Fireglow,
    Lamp,
    CopperOre,
    CopperIngot,
    CopperSword,
}

impl Display for Item {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                Self::Wood => "Wood",
                Self::Fireglow => "Fireglow",
                Self::Lamp => "Lamp",
                Self::CopperOre => "Copper Ore",
                Self::CopperIngot => "Copper Ingot",
                Self::CopperSword => "Copper Sword",
            }
        )
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq)]
pub struct ItemStack {
    pub item: Item,
    pub amount: u32,
}

impl Display for ItemStack {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} x{}", self.item, self.amount)
    }
}

```


=== common::lib.rs

```pretty-rs
// lib.rs
pub mod item;
pub mod net;

use std::ops::Deref;

pub trait Observer<T> {
    fn notify(&self, old: &T, new: &T);
}

pub struct Observable<T: Clone> {
    inner: T,
    observers: Vec<Box<dyn Observer<T>>>,
}

impl<T: Clone> Observable<T> {
    pub fn new(inner: T) -> Self {
        Self {
            inner,
            observers: Vec::new(),
        }
    }

    pub fn register(&mut self, observer: Box<dyn Observer<T>>) {
        self.observers.push(observer)
    }

    pub fn run<F: FnOnce(&mut T)>(&mut self, predicate: F) {
        let old = self.inner.clone();
        predicate(&mut self.inner);
        self.observers
            .iter()
            .for_each(|observer| observer.notify(&old, &self.inner))
    }

    // Chose to do this instead of DerefMut to be more verbose about the fact observers won't be
    // triggered
    pub fn run_silent<F: FnOnce(&mut T)>(&mut self, predicate: F) {
        predicate(&mut self.inner);
    }
}

impl<T: Clone> Deref for Observable<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

```


=== components::inventory.rs

```pretty-rs
// inventory.rs
use glam::{UVec2, Vec4};

use super::components::{Container, HAlign, Padding, Text, VList};
use crate::{
    data::inventory::Inventory,
    ui::{self, Element, Rectangle, Region, SizeConstraints},
};

pub type Component = Container<Padding<VList<Text>>>;

impl Component {
    pub fn new(inventory: &Inventory) -> Self {
        let text = inventory
            .get_items()
            .iter()
            .map(|stack| Text {
                color: ui::color::get_highlight(),
                content: format!("{}", stack),
            })
            .collect::<Vec<Text>>();
        let vlist = VList {
            children: text,
            separation: 3,
            align: HAlign::Left,
        };
        let padding = Padding::new_uniform(vlist, 2);
        Self {
            child: padding,
            color: ui::color::get_background(),
            border_radius: 1,
            border_color: ui::color::get_highlight(),
        }
    }
}

```


=== aetheria::socket.rs

```pretty-rs
// socket.rs
use common::net;
use std::{
    net::UdpSocket,
    ops::{Deref, DerefMut},
};

#[derive(thiserror::Error, Debug)]
pub enum PacketSendError {
    #[error("Error sending packet")]
    IOError(#[from] std::io::Error),
    #[error("Error encoding packet")]
    PostcardError(#[from] postcard::Error),
}

pub struct Socket {
    inner: UdpSocket,
}

impl Socket {
    pub fn send(&self, packet: &net::server::Packet) -> Result<(), PacketSendError> {
        let bytes = postcard::to_stdvec(packet)?;
        self.inner.send(&bytes)?;
        Ok(())
    }
}

impl Deref for Socket {
    type Target = UdpSocket;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl DerefMut for Socket {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl From<UdpSocket> for Socket {
    fn from(value: UdpSocket) -> Self {
        Self { inner: value }
    }
}

```


=== entities::grass.rs

```pretty-rs
// grass.rs
use std::sync::{Arc, Mutex};

use ash::vk;
use assets::{ModelRegistry, Transform};
use glam::Vec3;

use crate::{
    renderer::Renderer,
    systems::{
        render::{RenderObject, Renderable},
        Systems,
    },
};

pub struct Grass {
    pub grass: RenderObject,
}

impl Grass {
    pub fn new(
        renderer: &mut Renderer,
        systems: &mut Systems,
        model_registry: &mut ModelRegistry,
        transform: Transform,
    ) -> Result<Arc<Mutex<Self>>, vk::Result> {
        let grass = RenderObject {
            model: model_registry.load("grass.glb"),
            transform,
        };
        let grass = Arc::new(Mutex::new(Self { grass }));
        systems.render.add(grass.clone());
        Ok(grass)
    }
}

impl Renderable for Grass {
    fn get_objects(&self) -> Vec<RenderObject> {
        vec![self.grass.clone()]
    }
}

```


=== data::mod.rs

```pretty-rs
// mod.rs
use common::item::ItemStack;

pub mod inventory;

#[derive(Clone, Debug)]
pub struct Recipe {
    pub ingredients: Vec<ItemStack>,
    pub outputs: Vec<ItemStack>,
}

impl Recipe {
    pub fn has_ingredients(&self, inventory: &inventory::Inventory) -> bool {
        self.ingredients
            .iter()
            .map(|ingredient| {
                ingredient.amount
                    <= inventory
                        .get_items()
                        .iter()
                        .find(|stack| stack.item == ingredient.item)
                        .map(|stack| stack.amount)
                        .unwrap_or(0)
            })
            .all(|x| x)
    }
}

pub struct Data {
    pub inventory: inventory::Inventory,
    pub current_recipe: Option<Recipe>,
    pub recipe_selections: Option<Vec<Recipe>>,
}

```


=== aetheria::Cargo.toml

```toml
// Cargo.toml
[package]
name = "aetheria"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies]
assets = { path = "../assets" }
gltf = { path = "../gltf" }
hecs = { path = "../hecs" }
vulkan = { path = "../vulkan" }
common = { path = "../common" }

ash = { version = "0.37.2", features = ["linked"] }
winit = "0.28"
cstr = "0.2.11"
bytemuck = { version = "1.13", features = ["derive"] }
tracing = "0.1"
tracing-subscriber = "0.3"
gpu-allocator = "0.22"
glam = { version = "0.24", features = ["bytemuck"] }
qoi = "0.4"
rand = "0.8.5"
anyhow = "1.0.71"
num-traits = "0.2.15"
postcard = { version = "1.0.6", features = ["use-std"] }
thiserror = "1.0.44"
dialog = "0.3.0"
uuid = { version = "1.4.1", features = ["v4", "fast-rng"] }

```


=== arbiter::Cargo.toml

```toml
// Cargo.toml
[package]
name = "arbiter"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies]
common = { path = "../common" }

anyhow = "1.0.71"
num-derive = "0.4.0"
num-traits = "0.2.15"
tracing = "0.1.37"
tracing-subscriber = "0.3.17"
glam = { version = "0.24.1", features = ["bytemuck"] }
bytemuck = "1.13.1"
postcard = { version = "1.0.6", features = ["use-std"] }
thiserror = "1.0.44"
async-std = { version = "1.12.0", features = ["attributes"] }
sqlx = { version = "0.7.1", features = ["runtime-async-std", "sqlite"] }


```


=== aetheria::macros.rs

```pretty-rs
// macros.rs
#[repr(C)] // guarantee 'bytes' comes after '_align'
pub struct AlignedAs<Align, Bytes: ?Sized> {
    pub _align: [Align; 0],
    pub bytes: Bytes,
}

#[macro_export]
macro_rules! include_bytes_align_as {
    ($align_ty:ty, $path:literal) => {{
        // const block expression to encapsulate the static
        use $crate::macros::AlignedAs;

        // this assignment is made possible by CoerceUnsized
        static ALIGNED: &AlignedAs<$align_ty, [u8]> = &AlignedAs {
            _align: [],
            bytes: *include_bytes!($path),
        };

        &ALIGNED.bytes
    }};
}

```


=== components::ui.rs

```pretty-rs
// ui.rs
use glam::UVec2;

use crate::ui::{Rectangle, Element, SizeConstraints, Region};

use super::{inventory, craft};

pub struct UI<'a> {
   pub inventory: bool,
   pub craft: bool
}

impl UI<'_> {
    pub fn new() -> Self {
        Self {
            inventory: None,
            craft: None
        }
    }
}

impl Element for UI {
    fn layout(&mut self, constraint: SizeConstraints) -> UVec2 {
        UVec2::new(constraint.max.x, constraint.max.y)
    }

    fn paint(&mut self, region: Region, scene: &mut Vec<Rectangle>) {
        if self.inventory
    }
} ff ff

```


=== macros::lib.rs

```pretty-rs
// lib.rs
use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput};

#[proc_macro_derive(Entity)]
pub fn entity_derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);

    let name = input.ident;
    let gen = quote! {
        impl Entity for #name {}
    };

    gen.into()
}

#[proc_macro_derive(Scene)]
pub fn scene_derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);

    TokenStream::new()
}

```


=== systems::ui.rs

```pretty-rs
// ui.rs
use std::sync::{Weak, Mutex, Arc};

pub struct System {
    generators: Vec<Weak<Mutex<dyn UIGenerator>>>
}

impl System {
    pub fn new() -> Self {
        Self { generators: Vec::new() }
    }

    pub fn add<T: UIGenerator + Sized + 'static>(&mut self, generator: Arc<Mutex<T>>) {
        self.generators.push(Arc::downgrade(
            &(generator as Arc<Mutex<dyn UIGenerator>>),
        ))
    }
}

pub trait UIGenerator {
    fn generate() ff
} ff

```


=== hecs::lib.rs

```pretty-rs
// lib.rs
use std::any::{Any, TypeId};

pub use hecs_macros::*;

pub trait Scene {
    fn tick(&mut self);
    fn load() -> Self;
}

pub trait Entity: Any {}

pub trait System<T: Entity> {
    fn filter(entity: &dyn Entity) -> bool {
        println!(
            "Looking for {:?}, found {:?}",
            TypeId::of::<T>(),
            entity.type_id()
        );

        entity.type_id() == TypeId::of::<T>()
    }

    fn run(&mut self, entity: &mut T);
}

```


=== assets::Cargo.toml

```toml
// Cargo.toml
[package]
name = "assets"
version = "0.1.0"
edition = "2021"

[build-dependencies]
shaderc = "0.8"
image = "0.24"
qoi = "0.4"

[dependencies]
vulkan = { path = "../vulkan" }
gltf = { path = "../gltf" }

ash = "0.37.2"
bytemuck = { version = "1.13", features = ["derive"] }
tracing = "0.1"
tobj = "4.0.0"
glam = { version = "0.24", features = ["bytemuck"] }
uuid = { version = "1.4.1", features = ["v4", "fast-rng"] }

```


=== common::Cargo.toml

```toml
// Cargo.toml
[package]
name = "common"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies]
num-derive = "0.4.0"
num-traits = "0.2.15"
bytemuck = "1.13"
glam = { version = "0.24", features = ["serde"] }
thiserror = "1.0.44"
postcard = "1.0.6"
serde = { version = "1.0.180", features = ["derive"] }

```


=== vulkan::Cargo.toml

```toml
// Cargo.toml
[package]
name = "vulkan"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies]
ash = { version="0.37", features = ["linked"] }
winit = "0.28"
cstr = "0.2.11"
bytemuck = "1.13"
tracing = "0.1"
gpu-allocator = "0.22"
glam = { version = "0.23", features = ["bytemuck"] }
qoi = "0.4"

```


=== entities::mod.rs

```pretty-rs
// mod.rs
mod firefly;
pub use firefly::Firefly;

mod grass;
pub use grass::Grass;

mod player;
pub use player::Player;

mod sun;
pub use sun::Sun;

mod tree;
pub use tree::Tree;

mod furnace;
pub use furnace::Furnace;

mod crafting_bench;
pub use crafting_bench::CraftingBench;

mod copper_ore;
pub use copper_ore::CopperOre;

```


=== systems::mod.rs

```pretty-rs
// mod.rs
use glam::Vec3;

pub mod interact;
pub mod render;

pub struct Systems<'a> {
    pub interact: &'a mut interact::System,
    pub render: &'a mut render::System,
}

pub trait Named {
    fn get_name(&self) -> String;
}

pub trait Positioned {
    fn get_position(&self) -> Vec3;
}

```


=== gltf::Cargo.toml

```toml
// Cargo.toml
[package]
name = "gltf"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies]
serde = { version = "1.0", features = ["derive"] }
serde_json = "1.0"
bytemuck = "1.13"
serde_repr = "0.1"

```


=== macros::Cargo.toml

```toml
// Cargo.toml
[package]
name = "hecs-macros"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies]
syn = "1.0"
quote = "1.0"

[lib]
proc-macro = true

```


=== hecs::Cargo.toml

```toml
// Cargo.toml
[package]
name = "hecs"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies]
hecs-macros = { path = "macros" }
```


=== ::Cargo.toml

```toml
// Cargo.toml
[workspace]
default-members = ["aetheria"]
members = [
  "assets",
  "aetheria",
  "arbiter",
  "common",
  "gltf",
  "hecs",
  "hecs/macros",
  "vulkan"
]
resolver = "2"

[profile.release]
debug = true

```


=== arbiter::build.rs

```pretty-rs
// build.rs
// generated by `sqlx migrate build-script`
fn main() {
    // trigger recompilation when a new migration is added
    println!("cargo:rerun-if-changed=migrations");
}

```


=== scenes::mod.rs

```pretty-rs
// mod.rs
mod fireflies;
pub use fireflies::Fireflies;

mod root;
pub use root::RootScene;

mod trees;
pub use trees::Trees;

mod ores;
pub use ores::Ores;

```


=== components::mod.rs

```pretty-rs
// mod.rs
pub mod components;
pub mod craft;
pub mod interact;
pub mod inventory;
pub mod recipe_selector;

```

