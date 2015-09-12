Cinder-OculusRift
==================

This is yet another Oculus Rift block for Cinder, and a collaboration between [Paul Houx](https://github.com/paulhoux) & myself. *Thanks Paul for all the initial work!* It currently uses version 0.7 of the Oculus SDK, and the latest glNext Cinder version. Support for DK2 Windows.

Samples
-----------------
* Basic Sample: Features the standard use case scenario, with all the most common functionalities.
![BasicSample](https://dl.dropboxusercontent.com/u/29102565/oculus/basicsample.png)

* Spherical Stereo: Maps a sample Arnold 360 texture to a sphere, in stereo. Useful for both live-action & pre-rendered CG content.
![SphericalStereo](https://dl.dropboxusercontent.com/u/29102565/oculus/sphericalstereo.png)

* Instanced Stereo: Similar to the Basic Sample, only it uses instanced stereo rendering as described [here](https://docs.google.com/presentation/d/19x9XDjUvkW_9gsfsMQzt3hZbRNziVsoCEHOn4AercAc/edit).

Usage
-----------------
First, initialize the rift manager in prepareSettings.

```
using namespace hmd;

RiftManager::initialize();
```

Create an Oculus Rift instance:
```
OculusRiftRef	mRift;
...
mRift = OculusRift::create();
```

The OculusRift class has two cameras: a convenience host camera controlling the overall head position & orientation, and an active eye camera which is updated by the SDK according to the tracked position & orientation. Their transformations are composed and can be queried via the `hmd::OculusRift` interface.


In the update()** loop, call bind to render the scene to the rift's framebuffers. Iterate over each eye (enabling it) and draw your scene as follows:
```
ScopedRiftBuffer bind{ mRift };
...
for( auto eye : mRift.getEyes() ) {
		mRift.enableEye( eye );
		drawScene();
}
```

_*The draw code must be called from the update() loop due to a [bug in the latest SDK](https://forums.oculus.com/viewtopic.php?f=20&t=25930)._

To avoid judder, make sure you hit **75fps**!


