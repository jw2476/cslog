#import "@preview/codelst:0.0.3": sourcecode

#set page(height: auto, fill: rgb(17, 17, 27))
#set text(fill: rgb(202, 211, 245), font: "JetBrains Mono")

// #set page(height: auto)
#set heading(numbering: "1.")

#show raw.where(block: false): code => box[
        #set text(size: 10pt, fill: rgb(250, 179, 135), font: "JetBrains Mono")

        #box(
            stroke: 1pt + rgb(69, 71, 90),
            fill: rgb(30, 30, 46),
            radius: 3pt,
            outset: (x: 2pt, y: 4pt),
            inset: (x: 2pt),
            code
        )
]

#show raw.where(lang: "pretty-rs"): code => block[
    #set text(size: 11pt, fill: rgb(202, 211, 245), font: "JetBrains Mono")

    #block(            
        stroke: 1pt + rgb(69, 71, 90),
        fill: rgb(30, 30, 46), 
        radius: 4pt, 
        inset: (x: 5pt, y: 5pt), 
        sourcecode(
            numbers-style: (i) => text(font: "JetBrains Mono", size: 9pt, fill: rgb(255, 255, 255), i),
            raw(code.text, lang: "rs")
        )

    )]

#set raw(
    theme: "Mocha.tmTheme"
)

#set table(stroke: rgb(137, 180, 250))

#show outline.entry: it => {
  if it.at("label", default: none) == <modified-entry> {
    it // prevent infinite recursion
  } else {
    [#outline.entry(
      it.level,
      it.element,
      it.body,
      [],  // remove fill
      []  // remove page number
    ) <modified-entry>]
  }
}

#outline(indent: auto)

= Introduction
MMOs (massively multiplayer online games) are a genre of video games focusing on connecting thousands of players in one central shared world where all players can interact and affect each other's experiences. Games from this genre include World of Warcraft, RuneScape, Final Fantasy XIV, Guild Wars 2 and many more, many of these games however were released around a decade ago and don't take advantage of modern computing. Furthermore, many of these games struggle to stay relevant, especially to younger audiences due to the outdated graphics and systems. My project is a game that hopes to explore what a possible MMORPG taking advantage of modern technologies could look like, focusing on immersion, simulation and player interaction.

= Analysis
== Why Computation

Computers are well suited to MMO servers as they can simulate a world with thousands of players in real-time while generating content for players to explore, a human wouldn't be able to keep up with so many individuals at once. Computers are also well suited for MMO clients as modern computer graphics can render realistic environments real-time. In addition computers are good at communicating with each other quickly and from long distances which is a needed feature of an MMO client otherwise it cannot talk to the server, humans are too slow at communication for it to be efficient on a large scale.

== Stakeholders

The main stakeholders for my game would be the players as they are the target audience. This group can be split into two main demographics: people who are new to MMOs, and those who are coming from an existing MMO. To deal with this range of experience levels, I will need to make sure there is a good set of tutorials to make sure players new to the genre can understand the game. For gamers more experienced with MMOs, I will need to look at existing games and implement similar feature sets that these players will be expecting, while still adding something new to the game to make it stand out. I'll also need to be considering both casual and competitive players, and make mechanics and content to keep both happy.

To get a representative sample for each demographic, I will be talking to people from varying experience ranges with the genre. In addition I will be sending out testing samples throughout the development process, asking both groups for feedback.

== Research
=== Final Fantasy XIV
Final Fantasy XIV(FFXIV) is an MMORPG released by Square Enix in 2010. The game revolves around its single-player story which is mainly comprised of voice-acted cutscenes with various NPC characters. The story unlocks most of the other mechanics in the game such as gathering and crafting, dungeons, raids and mounts. Gathering and crafting in FFXIV is unique thanks to its systems: players are given a series of abilities that either increase Progression, Quality, or increase the potential of other abilities. Once Progression reaches 100% the item is gathered/ crafted, the Quality value at this time determines the item's chance to be High Quality, meaning the item will be valued higher. This creates a fun minigame for gathering and crafting which helps to disrupt the monotony of many video game gathering/ crafting systems, this is something I am hoping to replicate in my game.

=== Guild Wars 2
Guild Wars 2 is an MMORPG developed by ArenaNet in 2012, compared to Final Fantasy XIV the game is much more open ended in its progression, instead of a central story the player is given a level which determines the content they can access, XP can be acquired from many sources such as crafting, questing, PvP and exploration. Guild Wars 2 has had three expansions added to it: Heart of Thorns, Path of Fire and End of Dragons, each of these expansions have added classes, specialisations, new chapters to the Living World and new zones for players to explore. Heart of Thorns also changed the progression system from level-based to the Mastery System. The Mastery System is a huge tree of achievements, tasks and challenges that each reward the player with items and Mastery Levels which can be used like skill points in many games. This creates a completely open ended, horizontal progression system that rewards players for exploring the game while allowing them to pick and choose what rewards they want, this is something I am hoping to base my game's progression on.

=== Palia
Palia is an unreleased MMO being developed by Singularity 6, it focuses on providing a more casual, laid-back experience that many other MMORPGs which can fixate on combat and a grand story when many players just want to craft items for their house, farm crops and trade with their friends. Not much has been released about Palia yet, but I would like to try and incorporate this focus on more casual features into the final game.

=== Spiritfarer
Spiritfarer is an 2-player indie RPG developed by Thunder Lotus Games. Its a game about running a boat for spirits to live on while they prepare to move on into the afterlife, the core gameplay is about making sure these spirits are happy, well-fed and housed by gathering and crafting materials to be used for the construction of the boat. While the story is fantastic, its not something I am planning to focus on. The part that interests me is the minigames, there's one for foraging, mining, smithing, smelting, cutting down trees, weaving, cooking and more, each of these minigames are fun and engaging and your performance in the minigame determines the yield of the output, if your timing is off when your cutting a tree, you'll get less wood, if you time a pickaxe swing badly, you'll get less ore, etc. Spiritfarer is going to be main inspiration for minigames for otherwise tedious mechanics.

=== Stardew Valley
Stardew Valley made by ConcernedApe presents itself as a basic indie farming game, and while the farming part of the game is great, the part where it is really fantastic is making the player feel like they are part of the game world. The village throws festivals, has birthday parties for the NPCs and as the player becomes friendlier with the villagers they start getting invited to these events, they get integrated into the village and made to feel at home. This personal connection with the player is what many feel Stardew Valley is all about, and something I would like to try and emulate in the final game.

=== Ashes of Creation
Ashes of Creation is an unreleased MMORPG being developed by Intrepid Studios. It is focusing on making a dynamic world where player actions result in huge changes to the game's world. The world is split up into areas called nodes, eazch node has a development level which determines the type and level of buildings players are allowed to build in a city. Nodes also have abundances of certain resources and scarcity of others, for example one node may have a surplus of food but lack ore, so must trade with another nearby node for those resources to keep its economy alive. Events like this are called emergent gameplay as they emerged from other mechanics the game developers designed, rather than being designed directly. This emergent gameplay is something I am looking to achieve in the final game.

=== TODO: Path of Eternity
=== TODO: Runescape

== Essential Features 


== Limitations
Due to limitations in time and budget, the game will not target smartphones or consoles, nor macOS as I don't own a Mac, however both Windows and Linux should be supported. In addition, I'm not great at art, so I will use a simplistic, low-poly style so the assets are simpler to create, I'm also hoping to use music licenced under Creative Commons so I can avoid making my own, I'm more interesting in the game design and programming.

Also due to the goal of making the game run on as many devices as possible, I'm not going to be able to expect cutting-edge hardware, the game will need to be performant and optimised for older machines, my test for this will be my 6 year old laptop. 

Finally I don't have a large budget for servers, so the server code needs to be well optimised and efficient so it can run on a machine like a Raspberry Pi.

== Success Criteria
To succeed my game will need to:
- Run on as many devices as possible
- Have low-latency, reliable networking
- Have intuitive, fun mechanics and UI
- Value the players time
- Have many mechanics to allow players to play the parts they enjoy while interacting with the rest of the world resulting in emergent gameplay
- Have an immersive world-wide story delivered through events and cutscenes

Many of these criteria are very subjective, so there will be phases of playtesting with stakeholders during development where the stakeholders get to play the game for a while and will evaluate it against this criteria. I will also be evaluating against this criteria when designing mechanics and systems.

== Requirements
=== Hardware
- A computer with standard peripherals like keyboard, mouse/trackpad, etc
- A GPU capable of running Vulkan, this is mainly driver dependant unless the GPU is 10+ years old
- More concrete requirements will be decided when the game is closer to completion

=== Software
Because the game needs to be able to run on as many devices as possible, I've tried to keep the requirements as basic as possible: 
- Up-to-date graphics driver will be needed as I'll be using modern graphics APIs like Vulkan
- Windows 10 or Linux

= Design

== Gameplay Loops
The majority of the game can be split into different gameplay loops:
#image("design/gameplay_loops.png")

=== Gathering
The gathering system will be responsible for the collection of most resources for crafting and trading, from wood to ores to passive monster drops.

=== Crafting

=== Combat

=== Trading

#include "devlogs/mod.typ"

$ integral^(sum_(n = 0)^infinity ((-1)^n)/(n+1))_(sum_(n = 0)^infinity sqrt(n) - sqrt(n+1)) (lim_(t -> infinity) (1 + 1/e^t)^(e^t))^(d/"dx" ( x^2/(sin^2x + cos^2x))) $