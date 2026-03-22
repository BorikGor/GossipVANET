# Stability in Vehicular Ad Hoc Networks (VANETs)
### Adaptive Gossip-Based Dissemination Protocol

## Overview
This repository contains the implementation and experimental evaluation of an adaptive gossip-based communication protocol designed to improve message delivery stability in Vehicular Ad Hoc Networks (VANETs).

The project focuses on decentralized, lightweight dissemination mechanisms that operate under highly dynamic network conditions, without relying on centralized infrastructure or computationally intensive algorithms.

---

## Academic Context
This work is part of a **Research Project** submitted in partial fulfilment of the requirements for the degree of **Master of Engineering (Coursework)** at **Auckland University of Technology (AUT)**.

- Programme: Master of Engineering (180 points)
- Pathway: Coursework
- Output type: Research Project
- Institution: Auckland University of Technology (AUT)
- Author: Boris Gor

---

## Research Motivation
VANETs are characterized by rapid topology changes, intermittent connectivity, and high node mobility. Traditional MANET routing protocols often fail under such conditions due to route instability, fragmentation, and excessive control overhead.

This project explores **epidemic (gossip-based) dissemination algorithms** as a viable alternative, aiming to:
- Improve stability under fragmentation
- Reduce redundant transmissions
- Maintain high delivery ratios
- Operate on resource-constrained embedded platforms

---

## Key Contributions
- Design of an adaptive gossip-based dissemination protocol tailored for vehicular environments
- Integration of vehicular context (speed, direction, density) into dissemination logic
- Emergency event propagation without reliance on roadside infrastructure
- Comparative evaluation against baseline dissemination approaches

---

## Baseline Protocols
The proposed protocol is evaluated against the following baseline methods:
- **Flooding** (upper-bound delivery, high overhead)
- **Trickle algorithm**
- **RPL (Routing Protocol for Low-Power and Lossy Networks)**

All protocols are tested under identical mobility and network conditions.

---

## Simulation Environment
- Operating system: **Contiki-NG**
- Simulator: **Cooja**
- Mobility model: **SUMO-generated vehicular traces**
- Node types:
  - Mobile nodes (vehicles)
  - Stationary nodes (RSUs / sinks)

---

## Project Structure
```text
/firmware        Embedded firmware (mobile nodes, RSUs)
/simulation      Cooja simulation files
/scripts         Cooja scripts and automation
/results         Experimental results and logs
/docs            Project proposal and report drafts
README.md
