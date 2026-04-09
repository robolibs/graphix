# Introduction

`graphix` is a Rust library for practical graph and factor-graph workflows.

It is built around three main pieces:

- `graphix::vertex` for graph containers and graph algorithms
- `graphix::vertex::spatial` for nearest-neighbor and graph-building utilities on top of 2D geometry
- `graphix::factor` for scalar and SE2 factor-graph optimization

The intended use case is not "generic low-level graph building blocks only". The library is aimed at workflows such as:

- building a spatial graph from points
- finding routes through that graph
- turning those routes into pose-graph seeds
- optimizing the resulting factor graph

This book is a usage guide. It focuses on the APIs you are expected to use directly.
