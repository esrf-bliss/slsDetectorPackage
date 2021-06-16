# Generic Frame Assembler library

The aims of the library is to propose a framework to reconstruct frames from network packets (UDP datagrams) given transfer properties (e.g. packet definition) and detector properties (e.g. detector assembly).

## Concepts

### Basics

#### Pixel

 - `Pixel`

#### Point

 - `XY`

### Network

#### Packet

 - `Packet`

#### Packet block

#### Packet container

#### Packet stream

### Detector Geometry

#### Geometry description

 - `ChipPixels`
 - `ChipGap`
 - `IfaceChips`
 - `RecvIfaces`
 - `ModRecvFlip`
 - `ModRecvs`
 - `ModGap`

### Frame Reconstruction

#### FrameAssembler

## API

`CreateFrameAssembler()`

returns

```
class FrameAssembler {
  public:
    virtual bool assembleFrame(AnyPacketBlockPtr &&block, RecvHeader *header,
                               char *buf) = 0;

    virtual FrameDims getAssembledFrameDims = 0;
};
```

with

```
struct FrameDims {
    slsDetectorDefs::xy dim;
    int size;
};
```

## Defining detectors

Each detectors need to define the possible configuration that it support in terms of pixels, network topology, geometry...

### Network

Define `Packet` for the detector, the *layout* of the packets.
Define `StreamData` for the detector, the *order* of the packets.

### Geometry

Define `GeomData` for the detector.

### Frame assembler
