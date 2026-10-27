# Upstream provenance

This project is a GitHub fork of:

- https://github.com/qalby-tech/embryo_lab_chess_so101
- Starting revision: `bd972d2f7db450813f50f451e8edacc108841737`
- Fork: https://github.com/ForceConstant/robot-arm-sim

Upstream source, generated piece assets, contact-model assumptions and Git
history are retained. No new blanket license is applied to upstream content;
the upstream repository did not declare a top-level license at the starting
revision. Robot model provenance is described in upstream documentation and
the vendored assets. Fork-specific changes add a two-arm scene, a LAN web
control/viewing application and a Docker/Dockhand deployment.

The controller uses exact simulator state and tuned contact geometry. It is not
a demonstrated camera-only policy and has not been transferred to real hardware.
