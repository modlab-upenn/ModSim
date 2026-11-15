---
title: ModSim documentation
sidebar_label: Overview
slug: /
---
ModSim is an open-source framework for modular reconfigurable robots and multi-robot systems. These systems need support for robots that connect, disconnect, and form new assemblies while working together. Building this support on top of existing simulators often requires researchers to develop custom solutions and workarounds, with limited reuse across projects. As a result, different groups end up building similar tools from scratch.

To address this problem, ModSim provides a unified framework built around three main subsystems:

1. **RobotPack:** A format for describing a modular robotic system. A RobotPack combines mechanical assets, such as URDF models, with YAML descriptions of module types, connectors, and docking policies.

2. **Runtime:** The subsystem that maintains the world state, tracking module poses, connections, assemblies, and events as the system runs.

3. **Backend adapters:** The integration layer that connects ModSim to existing simulators and physics engines, such as MuJoCo and NVIDIA Isaac Sim.

This enables researchers to focus on development and inmplementaiton of new research ideas while building off a reusable framework with well established simulators.

The following documentation is for people learning to use ModSim, create RobotPacks, or understand and extend the codebase. The learning path begins with ModSim Studio to inspect, validate, and edit a small RobotPack, then introduces physics simulation through a visual docking example. A separate CLI guide covers terminal workflows.

We encourage and welcome contributions to the project, including new features, implementations of planners, and support for additional robotic systems. As research in modular robotics and multi-robot systems develops, we want ModSim to help bring these ideas into a common framework that the robotics community can use and build on.

The documentation is organized into four sections:

| Section | Use it to |
| --- | --- |
| Tutorials | Learn by completing a small working example. |
| How-to guides | Complete specific tasks, such as importing a URDF model or creating a RobotPack. |
| Explanation | Understand the codebase, architecture, and runtime behavior. |
| Reference | Look up schemas, CLI commands, contracts, and Python APIs. |

