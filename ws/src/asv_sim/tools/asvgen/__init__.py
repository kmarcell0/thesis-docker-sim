"""asvgen - parametric generator for the RoboBoat ASV Gazebo model.

Pipeline (one module per stage):

    config.py        read YAML, apply --set overrides, validate
    sensors.py       turn each sensor entry into SDF sensors, topics, frames, masses
    boat.py          assemble hull, deck, components, thruster pods and sensors
      mass.py        total mass, centre of gravity, inertia tensor
      hydrostatics.py draft, buoyancy centre, stability (same model as VRX)
    sdf_model.py     serialise the assembled boat to an SDF <model>
    analysis.py      design checks and sensor coverage metrics
    outputs.py       write model.sdf, bridge/frames YAML, report (txt + json)
    plot.py          optional layout figure
    cli.py           command line entry point

Helpers: geometry.py (rotations, poses), xml_utils.py (SDF/XML building).
"""

__version__ = '2.0.0'
