# Generic Frame Assembler library

The aims of the library is to propose a framework to reconstruct frames from network packets (UDP datagrams) given transfer properties (e.g. packet definition) and detector properties (e.g. detector assembly).

## Concepts

### Basics

#### Pixel

`Pixel`

#### Point

`XY`

### Network

#### Packet

`Packet`

#### FramePackets

#### Packet container

#### Packet stream

### Detector Geometry

#### 

### Frame Reconstruction

#### FrameAssembler


## Defining detectors

Each detectors need to define the possible configuration that it support in terms of pixels, network topology, geometry...

### Network

Define `Packet` for the detector, the *layout* of the packets.
Define `StreamData` for the detector, the *order* of the packets.

### Geometry

Define `GeomData` for the detector.

### Frame assembler
