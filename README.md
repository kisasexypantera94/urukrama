# Urukrama
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![example workflow](https://github.com/kisasexypantera94/urukrama/actions/workflows/c-cpp.yml/badge.svg)

Urukrama is a simple and efficient implementation of DiskANN algorithm, developed for fun and self-education.

## Setup
```bash
git submodule update --init --recursive
docker compose up -d
```
attach to the container and run
```
make release
```

## References
* **DiskANN: Fast Accurate Billion-point Nearest Neighbor Search on a Single Node.**
    Suhas Jayaram Subramanya, Rohan Kadekodi, Ravishankar Krishaswamy, and Harsha Vardhan Simhadri, 2019.
* **ParlayANN: Scalable and Deterministic Parallel Graph-Based Approximate Nearest Neighbor Search Algorithms**.
    Magdalen Dobson Manohar, Zheqi Shen, Guy E. Blelloch, Laxman Dhulipala, Yan Gu, Harsha Vardhan Simhadri, Yihan Sun, 2023.
