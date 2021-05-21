package pres_stp
  model fan_dp "Demonstration of the use of prescribedPressure"
    extends Modelica.Icons.Example;
    package Medium = Buildings.Media.Air;
    parameter Modelica.SIunits.MassFlowRate m_flow_nominal = 0.1 "Nominal mass flow rate";
    parameter Modelica.SIunits.PressureDifference dp_nominal = 100 "Nominal pressure difference";
    Buildings.Fluid.Sources.Boundary_pT sou(redeclare package Medium = Medium, nPorts = 1) "Source" annotation(
      Placement(visible = true, transformation(extent = {{-114, -70}, {-94, -50}}, rotation = 0)));
    Buildings.Fluid.Movers.FlowControlled_dp floConDpSystem(redeclare package Medium = Medium, allowFlowReversal = false, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, m_flow_nominal = m_flow_nominal, prescribeSystemPressure = true, use_inputFilter = false) "Dp controlled fan that sets pressure difference at remote point in the system" annotation(
      Placement(visible = true, transformation(extent = {{-84, -70}, {-64, -50}}, rotation = 0)));
    Buildings.Fluid.FixedResistances.PressureDrop heaCoi2(redeclare package Medium = Medium, m_flow_nominal = m_flow_nominal, dp_nominal = dp_nominal / 2) "Heating coil pressure drop" annotation(
      Placement(visible = true, transformation(extent = {{-44, -70}, {-24, -50}}, rotation = 0)));
    Buildings.Fluid.Sensors.RelativePressure senRelPre(redeclare package Medium = Medium) "Pressure difference across air system" annotation(
      Placement(transformation(extent = {{0, -30}, {20, -50}})));
    Buildings.Fluid.Sources.Boundary_pT sin(redeclare package Medium = Medium, nPorts = 1) "Sink" annotation(
      Placement(visible = true, transformation(extent = {{130, -70}, {110, -50}}, rotation = 0)));
    Buildings.Fluid.Actuators.Dampers.Exponential dam3(redeclare package Medium = Medium, from_dp = true, use_inputFilter = false, dpDamper_nominal = 10, m_flow_nominal = m_flow_nominal / 2) "Damper" annotation(
      Placement(visible = true, transformation(extent = {{48, -70}, {68, -50}}, rotation = 0)));
    Buildings.Fluid.MixingVolumes.MixingVolume zone3(redeclare package Medium = Medium, V = 50, m_flow_nominal = m_flow_nominal, nPorts = 3, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial) "Mixing volume" annotation(
      Placement(transformation(extent = {{80, -60}, {100, -40}})));
    Buildings.Fluid.FixedResistances.PressureDrop duct3(redeclare package Medium = Medium, dp_nominal = dp_nominal / 2, m_flow_nominal = m_flow_nominal / 2) "Duct pressure drop" annotation(
      Placement(visible = true, transformation(extent = {{-10, -70}, {10, -50}}, rotation = 0)));
  Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare package Medium = Medium) annotation(
      Placement(visible = true, transformation(origin = {28, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimPID pid(limitsAtInit = true, yMax = 1, yMin = 0)  annotation(
      Placement(visible = true, transformation(origin = {28, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput d_pressure annotation(
      Placement(visible = true, transformation(origin = {132, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {132, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Flow annotation(
      Placement(visible = true, transformation(origin = {134, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {132, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Power annotation(
      Placement(visible = true, transformation(origin = {128, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {128, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput damper_position annotation(
      Placement(visible = true, transformation(origin = {130, -8}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {130, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput pres_stp annotation(
      Placement(visible = true, transformation(origin = {-130, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-142, -100}, {-122, -80}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Flow_stp annotation(
      Placement(visible = true, transformation(origin = {-130, 112}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-140, 50}, {-120, 70}}, rotation = 0)));
  Modelica.Blocks.Continuous.Integrator integrator(k = 1 / 3600, use_reset = false) annotation(
      Placement(visible = true, transformation(origin = {-24, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput kWh annotation(
      Placement(visible = true, transformation(origin = {132, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {132, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(senRelPre.p_rel, floConDpSystem.dpMea) annotation(
      Line(points = {{10, -31}, {10, -12}, {-82, -12}, {-82, -48}}, color = {0, 0, 127}));
    connect(floConDpSystem.port_a, sou.ports[1]) annotation(
      Line(points = {{-84, -60}, {-94, -60}}, color = {0, 127, 255}));
    connect(heaCoi2.port_a, floConDpSystem.port_b) annotation(
      Line(points = {{-44, -60}, {-44, -60}, {-64, -60}}, color = {0, 127, 255}));
    connect(zone3.ports[1], sin.ports[1]) annotation(
      Line(points = {{87.3333, -60}, {110, -60}}, color = {0, 127, 255}));
    connect(dam3.port_b, zone3.ports[2]) annotation(
      Line(points = {{68, -60}, {90, -60}}, color = {0, 127, 255}));
    connect(senRelPre.port_b, zone3.ports[3]) annotation(
      Line(points = {{20, -40}, {74, -40}, {74, -60}, {92.6667, -60}}, color = {0, 127, 255}));
    connect(senRelPre.port_a, heaCoi2.port_b) annotation(
      Line(points = {{0, -40}, {-24, -40}, {-24, -60}}, color = {0, 127, 255}));
    connect(duct3.port_a, heaCoi2.port_b) annotation(
      Line(points = {{-10, -60}, {-24, -60}}, color = {0, 127, 255}));
    connect(duct3.port_b, senMasFlo.port_a) annotation(
      Line(points = {{10, -60}, {18, -60}}, color = {0, 127, 255}));
    connect(senMasFlo.port_b, dam3.port_a) annotation(
      Line(points = {{38, -60}, {48, -60}}, color = {0, 127, 255}));
    connect(pid.y, dam3.y) annotation(
      Line(points = {{39, 84}, {58, 84}, {58, -48}}, color = {0, 0, 127}));
    connect(senMasFlo.m_flow, pid.u_m) annotation(
      Line(points = {{28, -48}, {28, 72}}, color = {0, 0, 127}));
    connect(senRelPre.p_rel, d_pressure) annotation(
      Line(points = {{10, -30}, {10, 20}, {132, 20}}, color = {0, 0, 127}));
    connect(floConDpSystem.P, Power) annotation(
      Line(points = {{-62, -50}, {-52, -50}, {-52, 100}, {128, 100}}, color = {0, 0, 127}));
    connect(senMasFlo.m_flow, Flow) annotation(
      Line(points = {{28, -48}, {28, 66}, {134, 66}}, color = {0, 0, 127}));
    connect(Flow_stp, pid.u_s) annotation(
      Line(points = {{-130, 112}, {16, 112}, {16, 84}}, color = {0, 0, 127}));
    connect(pres_stp, floConDpSystem.dp_in) annotation(
      Line(points = {{-130, 80}, {-74, 80}, {-74, -48}}, color = {0, 0, 127}));
    connect(dam3.y_actual, damper_position) annotation(
      Line(points = {{64, -52}, {66, -52}, {66, -8}, {130, -8}}, color = {0, 0, 127}));
    connect(floConDpSystem.P, integrator.u) annotation(
      Line(points = {{-62, -50}, {-52, -50}, {-52, 42}, {-36, 42}}, color = {0, 0, 127}));
    connect(integrator.y, kWh) annotation(
      Line(points = {{-13, 42}, {132, 42}}, color = {0, 0, 127}));
    annotation(
      Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-120, -120}, {120, 120}})),
      experiment(StopTime = 1, Tolerance = 1e-06),
      __Dymola_Commands(file = "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Movers/Validation/FlowControlled_dpSystem.mos" "Simulate and plot"),
      Documentation(info = "<html>
  <p>
  This example demonstrates and tests the use of
  <a href=\"modelica://Buildings.Fluid.Movers.Validation.FlowControlled_dp\">
  Buildings.Fluid.Movers.Validation.FlowControlled_dp</a>
  movers that use parameter
  <code>prescribeSystemPressure</code>.
  </p>
  <p>
  The mass flow rates and actual pressure heads of the two configurations are compared.
  </p>
  </html>", revisions = "<html>
  <ul>
  <li>
  May 4 2017, by Filip Jorissen:<br/>
  First implementation.
  This is for
  <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/770\">#770</a>.
  </li>
  </ul>
  </html>"),
      Icon(coordinateSystem(extent = {{-120, -120}, {120, 120}}), graphics = {Ellipse(origin = {-3, -4}, fillColor = {0, 100, 199}, fillPattern = FillPattern.Sphere, extent = {{-103, 104}, {103, -104}}, endAngle = 360), Polygon(origin = {-18, -2}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.HorizontalCylinder, points = {{0, 50}, {0, -50}, {54, 0}, {0, 50}}), Rectangle(origin = {-2, -78}, fillColor = {0, 127, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, 16}, {100, -16}}), Ellipse(origin = {-20, -4}, fillColor = {0, 100, 199}, fillPattern = FillPattern.Sphere, extent = {{4, 16}, {36, -16}}, endAngle = 360)}));
  end fan_dp;
  
  model fan_dp_test "Demonstration of the use of prescribedPressure"
    extends Modelica.Icons.Example;
    package Medium = Buildings.Media.Air;
    parameter Modelica.SIunits.MassFlowRate m_flow_nominal = 0.1 "Nominal mass flow rate";
    parameter Modelica.SIunits.PressureDifference dp_nominal = 100 "Nominal pressure difference";
    Buildings.Fluid.Sources.Boundary_pT sou(redeclare package Medium = Medium, nPorts = 1) "Source" annotation(
      Placement(transformation(extent = {{-120, -10}, {-100, 10}})));
    Buildings.Fluid.Movers.FlowControlled_dp floConDpSystem(redeclare package Medium = Medium, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, allowFlowReversal = false, m_flow_nominal = 1, use_inputFilter = false, prescribeSystemPressure = true) "Dp controlled fan that sets pressure difference at remote point in the system" annotation(
      Placement(visible = true, transformation(extent = {{-84, -70}, {-64, -50}}, rotation = 0)));
    Buildings.Fluid.FixedResistances.PressureDrop heaCoi2(redeclare package Medium = Medium, m_flow_nominal = m_flow_nominal, dp_nominal = dp_nominal / 2) "Heating coil pressure drop" annotation(
      Placement(visible = true, transformation(extent = {{-44, -70}, {-24, -50}}, rotation = 0)));
    Buildings.Fluid.Sensors.RelativePressure senRelPre(redeclare package Medium = Medium) "Pressure difference across air system" annotation(
      Placement(transformation(extent = {{0, -30}, {20, -50}})));
    Buildings.Fluid.Sources.Boundary_pT sin(redeclare package Medium = Medium, nPorts = 1) "Sink" annotation(
      Placement(transformation(extent = {{120, -10}, {100, 10}})));
    Buildings.Fluid.Actuators.Dampers.Exponential dam3(redeclare package Medium = Medium, from_dp = true, use_inputFilter = false, dpDamper_nominal = 10, m_flow_nominal = m_flow_nominal / 2) "Damper" annotation(
      Placement(visible = true, transformation(extent = {{48, -70}, {68, -50}}, rotation = 0)));
    Buildings.Fluid.MixingVolumes.MixingVolume zone3(redeclare package Medium = Medium, V = 50, m_flow_nominal = m_flow_nominal, nPorts = 3, energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial) "Mixing volume" annotation(
      Placement(transformation(extent = {{80, -60}, {100, -40}})));
    Buildings.Fluid.FixedResistances.PressureDrop duct3(redeclare package Medium = Medium, dp_nominal = dp_nominal / 2, m_flow_nominal = m_flow_nominal / 2) "Duct pressure drop" annotation(
      Placement(visible = true, transformation(extent = {{-10, -70}, {10, -50}}, rotation = 0)));
  Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare package Medium = Medium) annotation(
      Placement(visible = true, transformation(origin = {28, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimPID pid(limitsAtInit = true, yMax = 1, yMin = 0)  annotation(
      Placement(visible = true, transformation(origin = {28, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput pres_stp annotation(
      Placement(visible = true, transformation(origin = {-130, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Flow_stp annotation(
      Placement(visible = true, transformation(origin = {-130, 112}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput d_pressure annotation(
      Placement(visible = true, transformation(origin = {130, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-46, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Flow annotation(
      Placement(visible = true, transformation(origin = {128, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-46, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Power annotation(
      Placement(visible = true, transformation(origin = {128, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-46, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(senRelPre.p_rel, floConDpSystem.dpMea) annotation(
      Line(points = {{10, -31}, {10, -12}, {-82, -12}, {-82, -48}}, color = {0, 0, 127}));
    connect(floConDpSystem.port_a, sou.ports[1]) annotation(
      Line(points = {{-84, -60}, {-100, -60}, {-100, -2}}, color = {0, 127, 255}));
    connect(heaCoi2.port_a, floConDpSystem.port_b) annotation(
      Line(points = {{-44, -60}, {-44, -60}, {-64, -60}}, color = {0, 127, 255}));
    connect(zone3.ports[1], sin.ports[1]) annotation(
      Line(points = {{87.3333, -60}, {100, -60}, {100, -1}}, color = {0, 127, 255}));
    connect(dam3.port_b, zone3.ports[2]) annotation(
      Line(points = {{68, -60}, {90, -60}}, color = {0, 127, 255}));
    connect(senRelPre.port_b, zone3.ports[3]) annotation(
      Line(points = {{20, -40}, {74, -40}, {74, -60}, {92.6667, -60}}, color = {0, 127, 255}));
    connect(senRelPre.port_a, heaCoi2.port_b) annotation(
      Line(points = {{0, -40}, {-24, -40}, {-24, -60}}, color = {0, 127, 255}));
    connect(duct3.port_a, heaCoi2.port_b) annotation(
      Line(points = {{-10, -60}, {-24, -60}}, color = {0, 127, 255}));
    connect(duct3.port_b, senMasFlo.port_a) annotation(
      Line(points = {{10, -60}, {18, -60}}, color = {0, 127, 255}));
    connect(senMasFlo.port_b, dam3.port_a) annotation(
      Line(points = {{38, -60}, {48, -60}}, color = {0, 127, 255}));
    connect(pid.y, dam3.y) annotation(
      Line(points = {{39, 84}, {58, 84}, {58, -48}}, color = {0, 0, 127}));
    connect(senMasFlo.m_flow, pid.u_m) annotation(
      Line(points = {{28, -48}, {28, 72}}, color = {0, 0, 127}));
    connect(senRelPre.p_rel, d_pressure) annotation(
      Line(points = {{10, -30}, {10, 34}, {130, 34}}, color = {0, 0, 127}));
    connect(floConDpSystem.P, Power) annotation(
      Line(points = {{-62, -50}, {-52, -50}, {-52, 100}, {128, 100}}, color = {0, 0, 127}));
    connect(senMasFlo.m_flow, Flow) annotation(
      Line(points = {{28, -48}, {28, 66}, {128, 66}}, color = {0, 0, 127}));
    connect(Flow_stp, pid.u_s) annotation(
      Line(points = {{-130, 112}, {-6, 112}, {-6, 84}, {16, 84}}, color = {0, 0, 127}));
  connect(pres_stp, floConDpSystem.dp_in) annotation(
      Line(points = {{-130, 80}, {-74, 80}, {-74, -48}}, color = {0, 0, 127}));
    annotation(
      Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-120, -120}, {120, 120}})),
      experiment(StopTime = 1, Tolerance = 1e-06),
      __Dymola_Commands(file = "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Movers/Validation/FlowControlled_dpSystem.mos" "Simulate and plot"),
      Documentation(info = "<html>
  <p>
  This example demonstrates and tests the use of
  <a href=\"modelica://Buildings.Fluid.Movers.Validation.FlowControlled_dp\">
  Buildings.Fluid.Movers.Validation.FlowControlled_dp</a>
  movers that use parameter
  <code>prescribeSystemPressure</code>.
  </p>
  <p>
  The mass flow rates and actual pressure heads of the two configurations are compared.
  </p>
  </html>", revisions = "<html>
  <ul>
  <li>
  May 4 2017, by Filip Jorissen:<br/>
  First implementation.
  This is for
  <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/770\">#770</a>.
  </li>
  </ul>
  </html>"),
      Icon(coordinateSystem(extent = {{-120, -120}, {120, 120}})));
  end fan_dp_test;

  model comp_dp
  pres_stp.fan_dp fan_dp_50 annotation(
      Placement(visible = true, transformation(origin = {-6, 76}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  pres_stp.fan_dp fan_dp_75 annotation(
      Placement(visible = true, transformation(origin = {-6, 42}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  pres_stp.fan_dp fan_dp_100 annotation(
      Placement(visible = true, transformation(origin = {-4, 8}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  pres_stp.fan_dp fan_dp_125 annotation(
      Placement(visible = true, transformation(origin = {-2, -26}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  pres_stp.fan_dp fan_dp_150 annotation(
      Placement(visible = true, transformation(origin = {-4, -60}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 50)  annotation(
      Placement(visible = true, transformation(origin = {-54, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = 75)  annotation(
      Placement(visible = true, transformation(origin = {-52, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const2(k = 100)  annotation(
      Placement(visible = true, transformation(origin = {-52, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const3(k = 125)  annotation(
      Placement(visible = true, transformation(origin = {-52, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const4(k = 150)  annotation(
      Placement(visible = true, transformation(origin = {-52, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp PSup(duration = 700, height = 0.04, offset = 0, startTime = 100) annotation(
      Placement(visible = true, transformation(extent = {{-124, 72}, {-104, 92}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp ramp(duration = 500, height = 100, offset = 50, startTime = 800) annotation(
      Placement(visible = true, transformation(extent = {{-82, -120}, {-62, -100}}, rotation = 0)));
  pres_stp.fan_dp fan_dp annotation(
      Placement(visible = true, transformation(origin = {-12, -98}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  equation
    connect(const.y, fan_dp_50.pres_stp) annotation(
      Line(points = {{-42, 66}, {-20, 66}, {-20, 68}}, color = {0, 0, 127}));
  connect(const1.y, fan_dp_75.pres_stp) annotation(
      Line(points = {{-40, 32}, {-19, 32}, {-19, 33}}, color = {0, 0, 127}));
  connect(const2.y, fan_dp_100.pres_stp) annotation(
      Line(points = {{-40, -2}, {-32, -2}, {-32, -1}, {-17, -1}}, color = {0, 0, 127}));
  connect(const3.y, fan_dp_125.pres_stp) annotation(
      Line(points = {{-40, -36}, {-32, -36}, {-32, -35}, {-15, -35}}, color = {0, 0, 127}));
  connect(const4.y, fan_dp_150.pres_stp) annotation(
      Line(points = {{-40, -74}, {-28, -74}, {-28, -69}, {-17, -69}}, color = {0, 0, 127}));
    connect(PSup.y, fan_dp_50.Flow_stp) annotation(
      Line(points = {{-102, 82}, {-20, 82}, {-20, 86}}, color = {0, 0, 127}));
  connect(PSup.y, fan_dp_75.Flow_stp) annotation(
      Line(points = {{-102, 82}, {-102, 48}, {-19, 48}}, color = {0, 0, 127}));
  connect(PSup.y, fan_dp_100.Flow_stp) annotation(
      Line(points = {{-102, 82}, {-102, 14}, {-17, 14}}, color = {0, 0, 127}));
  connect(PSup.y, fan_dp_125.Flow_stp) annotation(
      Line(points = {{-102, 82}, {-102, -20}, {-15, -20}}, color = {0, 0, 127}));
  connect(PSup.y, fan_dp_150.Flow_stp) annotation(
      Line(points = {{-102, 82}, {-100, 82}, {-100, -54}, {-17, -54}}, color = {0, 0, 127}));
  connect(ramp.y, fan_dp.pres_stp) annotation(
      Line(points = {{-61, -110}, {-44.5, -110}, {-44.5, -107}, {-25, -107}}, color = {0, 0, 127}));
  connect(PSup.y, fan_dp.Flow_stp) annotation(
      Line(points = {{-102, 82}, {-102, -92}, {-25, -92}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {40, 100}})));
  end comp_dp;
  
  model dp_flow_fan "Demonstration of the use of prescribedPressure"
    extends Modelica.Icons.Example;
    replaceable package Medium = Buildings.Media.Air;
    parameter Real  m_flow_nominal = 0.1 "Nominal mass flow rate";
    parameter Real  dp_nominal = 100 "Nominal pressure difference";
    Buildings.Fluid.Movers.FlowControlled_dp floConDpSystem(redeclare package Medium = Medium, allowFlowReversal = false, energyDynamics = Modelica.Fluid.Types.Dynamics.SteadyState, m_flow_nominal = m_flow_nominal, prescribeSystemPressure = true, use_inputFilter = false) "Dp controlled fan that sets pressure difference at remote point in the system" annotation(
      Placement(visible = true, transformation(extent = {{-84, -70}, {-64, -50}}, rotation = 0)));
    Buildings.Fluid.FixedResistances.PressureDrop heaCoi2(redeclare package Medium = Medium, m_flow_nominal = m_flow_nominal, dp_nominal = dp_nominal / 2) "Heating coil pressure drop" annotation(
      Placement(visible = true, transformation(extent = {{-44, -70}, {-24, -50}}, rotation = 0)));
    Buildings.Fluid.Sensors.RelativePressure senRelPre(redeclare package Medium = Medium) "Pressure difference across air system" annotation(
      Placement(transformation(extent = {{0, -30}, {20, -50}})));
    Buildings.Fluid.Actuators.Dampers.Exponential dam3(redeclare package Medium = Medium, from_dp = true, use_inputFilter = false, dpDamper_nominal = 10, m_flow_nominal = m_flow_nominal / 2) "Damper" annotation(
      Placement(visible = true, transformation(extent = {{48, -70}, {68, -50}}, rotation = 0)));
    Buildings.Fluid.FixedResistances.PressureDrop duct3(redeclare package Medium = Medium, dp_nominal = dp_nominal / 2, m_flow_nominal = m_flow_nominal / 2) "Duct pressure drop" annotation(
      Placement(visible = true, transformation(extent = {{-10, -70}, {10, -50}}, rotation = 0)));
  Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare package Medium = Medium) annotation(
      Placement(visible = true, transformation(origin = {28, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimPID pid(limitsAtInit = true, yMax = 1, yMin = 0)  annotation(
      Placement(visible = true, transformation(origin = {28, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Flow annotation(
      Placement(visible = true, transformation(origin = {134, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {132, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Power annotation(
      Placement(visible = true, transformation(origin = {128, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {128, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput pres_stp annotation(
      Placement(visible = true, transformation(origin = {-130, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, 130},extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput Flow_stp annotation(
      Placement(visible = true, transformation(origin = {-130, 112}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 130},extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium = Medium) annotation(
      Placement(visible = true, transformation(origin = {-120, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-120, -92}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium = Medium) annotation(
      Placement(visible = true, transformation(origin = {120, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {120, -92}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  equation
    connect(senRelPre.p_rel, floConDpSystem.dpMea) annotation(
      Line(points = {{10, -31}, {10, -12}, {-82, -12}, {-82, -48}}, color = {0, 0, 127}));
    connect(heaCoi2.port_a, floConDpSystem.port_b) annotation(
      Line(points = {{-44, -60}, {-44, -60}, {-64, -60}}, color = {0, 127, 255}));
    connect(senRelPre.port_a, heaCoi2.port_b) annotation(
      Line(points = {{0, -40}, {-24, -40}, {-24, -60}}, color = {0, 127, 255}));
    connect(duct3.port_a, heaCoi2.port_b) annotation(
      Line(points = {{-10, -60}, {-24, -60}}, color = {0, 127, 255}));
    connect(duct3.port_b, senMasFlo.port_a) annotation(
      Line(points = {{10, -60}, {18, -60}}, color = {0, 127, 255}));
    connect(senMasFlo.port_b, dam3.port_a) annotation(
      Line(points = {{38, -60}, {48, -60}}, color = {0, 127, 255}));
    connect(pid.y, dam3.y) annotation(
      Line(points = {{39, 84}, {58, 84}, {58, -48}}, color = {0, 0, 127}));
    connect(senMasFlo.m_flow, pid.u_m) annotation(
      Line(points = {{28, -48}, {28, 72}}, color = {0, 0, 127}));
    connect(floConDpSystem.P, Power) annotation(
      Line(points = {{-62, -50}, {-52, -50}, {-52, 100}, {128, 100}}, color = {0, 0, 127}));
    connect(senMasFlo.m_flow, Flow) annotation(
      Line(points = {{28, -48}, {28, 66}, {134, 66}}, color = {0, 0, 127}));
    connect(Flow_stp, pid.u_s) annotation(
      Line(points = {{-130, 112}, {16, 112}, {16, 84}}, color = {0, 0, 127}));
    connect(pres_stp, floConDpSystem.dp_in) annotation(
      Line(points = {{-130, 80}, {-74, 80}, {-74, -48}}, color = {0, 0, 127}));
    connect(senRelPre.port_b, dam3.port_b) annotation(
      Line(points = {{20, -40}, {68, -40}, {68, -60}}, color = {0, 127, 255}));
    connect(port_a, floConDpSystem.port_a) annotation(
      Line(points = {{-120, -62}, {-84, -62}, {-84, -60}}));
    connect(dam3.port_b, port_b) annotation(
      Line(points = {{68, -60}, {120, -60}}, color = {0, 127, 255}));
    annotation(
      Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-120, -120}, {120, 120}})),
      experiment(StopTime = 1, Tolerance = 1e-06),
      __Dymola_Commands(file = "modelica://Buildings/Resources/Scripts/Dymola/Fluid/Movers/Validation/FlowControlled_dpSystem.mos" "Simulate and plot"),
      Documentation(info = "<html>
  <p>
  This example demonstrates and tests the use of
  <a href=\"modelica://Buildings.Fluid.Movers.Validation.FlowControlled_dp\">
  Buildings.Fluid.Movers.Validation.FlowControlled_dp</a>
  movers that use parameter
  <code>prescribeSystemPressure</code>.
  </p>
  <p>
  The mass flow rates and actual pressure heads of the two configurations are compared.
  </p>
  </html>", revisions = "<html>
  <ul>
  <li>
  May 4 2017, by Filip Jorissen:<br/>
  First implementation.
  This is for
  <a href=\"https://github.com/ibpsa/modelica-ibpsa/issues/770\">#770</a>.
  </li>
  </ul>
  </html>"),
      Icon(coordinateSystem(extent = {{-120, -120}, {120, 120}}), graphics = {Ellipse(origin = {-3, -4}, fillColor = {0, 100, 199}, fillPattern = FillPattern.Sphere, extent = {{-103, 104}, {103, -104}}, endAngle = 360), Polygon(origin = {-18, -2}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.HorizontalCylinder, points = {{0, 50}, {0, -50}, {54, 0}, {0, 50}}), Rectangle(origin = {-7.10543e-14, -92}, fillColor = {0, 127, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-120, 16}, {120, -16}}), Ellipse(origin = {-20, -4}, fillColor = {0, 100, 199}, fillPattern = FillPattern.Sphere, extent = {{4, 16}, {36, -16}}, endAngle = 360)}));
  end dp_flow_fan;

  model q_load_solver
    Modelica.Blocks.Math.Feedback delta_temp annotation(
      Placement(visible = true, transformation(origin = {-58, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain(k = 1005) annotation(
      Placement(visible = true, transformation(origin = {-24, -6}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    Modelica.Blocks.Math.Division division annotation(
      Placement(visible = true, transformation(origin = {6, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput return_temp annotation(
      Placement(visible = true, transformation(origin = {-110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-120, 30}, {-100, 50}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput supl_temp annotation(
      Placement(visible = true, transformation(origin = {-110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-120, -52}, {-100, -32}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput q_load annotation(
      Placement(visible = true, transformation(origin = {0, 108}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {-30, 110},extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealOutput air_flow annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  connect(delta_temp.y, gain.u) annotation(
      Line(points = {{-49, -6}, {-31, -6}}, color = {0, 0, 127}));
  connect(division.u2, gain.y) annotation(
      Line(points = {{-6, -6}, {-17, -6}}, color = {0, 0, 127}));
  connect(return_temp, delta_temp.u1) annotation(
      Line(points = {{-110, 60}, {-84, 60}, {-84, -6}, {-66, -6}}, color = {0, 0, 127}));
  connect(supl_temp, delta_temp.u2) annotation(
      Line(points = {{-110, -40}, {-58, -40}, {-58, -14}}, color = {0, 0, 127}));
  connect(q_load, division.u1) annotation(
      Line(points = {{0, 108}, {0, 30}, {-14, 30}, {-14, 6}, {-6, 6}}, color = {0, 0, 127}));
  connect(division.y, air_flow) annotation(
      Line(points = {{17, 0}, {110, 0}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {40, 100}}), graphics = {Polygon(origin = {-30, 22}, fillColor = {233, 185, 110}, fillPattern = FillPattern.Backward, points = {{-2, 80}, {-30, 20}, {-70, 20}, {-48, -20}, {-70, -62}, {-10, -40}, {70, -20}, {10, 20}, {-2, 80}}, smooth = Smooth.Bezier), Polygon(origin = {-28, 20}, fillColor = {136, 138, 133}, fillPattern = FillPattern.Backward, points = {{-2, 80}, {-16, -10}, {-70, 20}, {-22, -18}, {-70, -62}, {-10, -20}, {70, -20}, {-6, -14}, {-2, 80}})}));
  end q_load_solver;

  model simple_coil
    replaceable package Medium = Buildings.Media.Air;
    parameter Real coil_air_V = 0.01 "air volume in coil m3";
    parameter Real coil_air_flow = 1 "nominal air mass flow in coil kg/s";
    Modelica.Blocks.Math.Gain gain1(k = -1) annotation(
      Placement(visible = true, transformation(origin = {-52, -20}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
    Buildings.Fluid.MixingVolumes.MixingVolume vol(redeclare package Medium = Medium, V = coil_air_V, m_flow_nominal = coil_air_flow, nPorts = 2) annotation(
      Placement(visible = true, transformation(origin = {-1, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain2(k = -1) annotation(
      Placement(visible = true, transformation(origin = {4, 26}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput supl_stp annotation(
      Placement(visible = true, transformation(origin = {-110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-81, 41}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput chill_load annotation(
      Placement(visible = true, transformation(origin = {112, 66}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput suppl_temp annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium = Medium, m_flow_nominal = 1) annotation(
      Placement(visible = true, transformation(origin = {44, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.LimPID pid(  Td = 1,Ti = 200, k = 10,limitsAtInit = true, yMax = 1000000, yMin = 0) annotation(
      Placement(visible = true, transformation(origin = {-38, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHea(alpha = 0) annotation(
      Placement(visible = true, transformation(extent = {{-38, -30}, {-18, -10}}, rotation = 0)));
    Modelica.Blocks.Math.Gain gain3(k = -1) annotation(
      Placement(visible = true, transformation(origin = {-76, 50}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium = Medium) annotation(
      Placement(visible = true, transformation(origin = {-104, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-114, -38}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium = Medium) annotation(
      Placement(visible = true, transformation(origin = {106, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {126, -38}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  equation
  connect(senTem.T, suppl_temp) annotation(
      Line(points = {{44, -39}, {71, -39}, {71, 0}, {110, 0}}, color = {0, 0, 127}));
  connect(gain2.u, senTem.T) annotation(
      Line(points = {{12, 26}, {44, 26}, {44, -39}}, color = {0, 0, 127}));
  connect(gain1.y, preHea.Q_flow) annotation(
      Line(points = {{-45, -20}, {-38, -20}}, color = {0, 0, 127}));
  connect(supl_stp, gain3.u) annotation(
      Line(points = {{-110, 50}, {-83, 50}}, color = {0, 0, 127}));
  connect(gain2.y, pid.u_m) annotation(
      Line(points = {{-2, 26}, {-38, 26}, {-38, 38}}, color = {0, 0, 127}));
  connect(pid.y, chill_load) annotation(
      Line(points = {{-27, 50}, {53.5, 50}, {53.5, 66}, {112, 66}}, color = {0, 0, 127}));
  connect(gain3.y, pid.u_s) annotation(
      Line(points = {{-69, 50}, {-50, 50}}, color = {0, 0, 127}));
  connect(preHea.port, vol.heatPort) annotation(
      Line(points = {{-18, -20}, {-11, -20}}, color = {191, 0, 0}));
  connect(senTem.port_b, port_b) annotation(
      Line(points = {{54, -50}, {106, -50}}, color = {0, 127, 255}));
  connect(pid.y, gain1.u) annotation(
      Line(points = {{-26, 50}, {-18, 50}, {-18, 2}, {-60, 2}, {-60, -20}}, color = {0, 0, 127}));
  connect(vol.ports[1], port_a) annotation(
      Line(points = {{0, -30}, {-104, -30}, {-104, -58}}, color = {0, 127, 255}));
  connect(vol.ports[2], senTem.port_a) annotation(
      Line(points = {{0, -30}, {34, -30}, {34, -50}}, color = {0, 127, 255}));
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(origin = {6, -38}, fillColor = {0, 127, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-120, 16}, {120, -16}}), Ellipse(origin = {8.08, -32.45}, lineColor = {85, 87, 83}, fillColor = {114, 159, 207}, fillPattern = FillPattern.CrossDiag, extent = {{-70.08, 70.45}, {70.08, -70.45}}, endAngle = 360), Polygon(origin = {6, -32}, lineColor = {52, 101, 164}, fillColor = {239, 41, 41}, fillPattern = FillPattern.HorizontalCylinder, points = {{-48, 50}, {50, 50}, {-50, -50}, {50, -50}, {0, 0}, {-48, 50}})}),
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}})));
  end simple_coil;
  
  model ahu_v2
    package Medium = Buildings.Media.Air;
    replaceable package MediumA = Buildings.Media.Air "Medium for air";
    parameter Real coil_air_V = 0.01 "air volume in coil m3";
    parameter Real coil_air_flow = 1 "nominal air mass flow in coil kg/s";
    parameter Real nom_air_dp = 150 "air pressure drop nominal";
    parameter Real nom_air_flow = 1 "nominal air mass flow in fan kg/s";
  
    Modelica.Blocks.Math.Min min annotation(
      Placement(visible = true, transformation(origin = {-92, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Sources.Boundary_pT sin(redeclare package Medium = MediumA, nPorts = 1) annotation(
      Placement(visible = true, transformation(extent = {{96, -46}, {76, -26}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput fan_power annotation(
      Placement(visible = true, transformation(origin = {108, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput air_flow annotation(
      Placement(visible = true, transformation(origin = {108, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput chill_load annotation(
      Placement(visible = true, transformation(origin = {108, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium = MediumA, m_flow_nominal = 1) annotation(
      Placement(visible = true, transformation(origin = {-54, 6}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput suppl_temp annotation(
      Placement(visible = true, transformation(origin = {112, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Sources.Boundary_pT bou(redeclare package Medium = MediumA, nPorts = 1, use_T_in = true) annotation(
      Placement(visible = true, transformation(origin = {-82, 16}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    pres_stp.dp_flow_fan dp_flow_fan(redeclare package Medium = MediumA, dp_nominal = nom_air_dp, m_flow_nominal = nom_air_flow) annotation(
      Placement(visible = true, transformation(origin = {-18, -2}, extent = {{-12, 12}, {12, -12}}, rotation = 0)));
    pres_stp.simple_coil simple_coil(redeclare package Medium = MediumA, coil_air_V = coil_air_V, coil_air_flow = nom_air_flow) annotation(
      Placement(visible = true, transformation(origin = {18, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    pres_stp.q_load_solver q_load_solver annotation(
      Placement(visible = true, transformation(origin = {-14, -60}, extent = {{-10, 10}, {4, -10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput supl_stp annotation(
      Placement(visible = true, transformation(origin = {-134, 92}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-81, 41}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput outdoor_air_temp annotation(
      Placement(visible = true, transformation(origin = {-134, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-71, 51}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput return_air_temp annotation(
      Placement(visible = true, transformation(origin = {-134, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-61, 61}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput press_stp annotation(
      Placement(visible = true, transformation(origin = {-136, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-71, 51}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput q_load annotation(
      Placement(visible = true, transformation(origin = {-136, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-61, 61}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
  equation
    connect(dp_flow_fan.Flow, air_flow) annotation(
      Line(points = {{-5, -5}, {26.2, -5}, {26.2, -64}, {108, -64}}, color = {0, 0, 127}));
    connect(dp_flow_fan.Power, fan_power) annotation(
      Line(points = {{-5, -10}, {13.5, -10}, {13.5, -86}, {108, -86}}, color = {0, 0, 127}));
    connect(dp_flow_fan.port_b, simple_coil.port_a) annotation(
      Line(points = {{-6, 7.2}, {7, 7.2}, {7, 8.2}}, color = {0, 127, 255}));
    connect(min.y, bou.T_in) annotation(
      Line(points = {{-81, 46}, {-78.5, 46}, {-78.5, 28}, {-78, 28}}, color = {0, 0, 127}));
  connect(q_load_solver.air_flow, dp_flow_fan.Flow_stp) annotation(
      Line(points = {{-9, -60}, {0, -60}, {0, -14}, {-12, -14}}, color = {0, 0, 127}));
    connect(simple_coil.suppl_temp, suppl_temp) annotation(
      Line(points = {{30, 20}, {68, 20}, {68, 42}, {112, 42}}, color = {0, 0, 127}));
    connect(simple_coil.chill_load, chill_load) annotation(
      Line(points = {{30, 14}, {108, 14}}, color = {0, 0, 127}));
    connect(bou.ports[1], senTem.port_a) annotation(
      Line(points = {{-82, 6}, {-64, 6}}, color = {0, 127, 255}));
    connect(senTem.port_b, dp_flow_fan.port_a) annotation(
      Line(points = {{-44, 6}, {-30, 6}, {-30, 8}}, color = {0, 127, 255}));
  connect(senTem.T, q_load_solver.return_temp) annotation(
      Line(points = {{-54, -4}, {-54, -64}, {-25, -64}}, color = {0, 0, 127}));
    connect(simple_coil.port_b, sin.ports[1]) annotation(
      Line(points = {{30, 8}, {40, 8}, {40, -36}, {76, -36}}, color = {0, 127, 255}));
  connect(q_load_solver.supl_temp, supl_stp) annotation(
      Line(points = {{-25, -56}, {-38, -56}, {-38, 92}, {-134, 92}}, color = {0, 0, 127}));
    connect(supl_stp, simple_coil.supl_stp) annotation(
      Line(points = {{-134, 92}, {-38, 92}, {-38, 16}, {10, 16}}, color = {0, 0, 127}));
    connect(return_air_temp, min.u1) annotation(
      Line(points = {{-134, 64}, {-118, 64}, {-118, 52}, {-104, 52}}, color = {0, 0, 127}));
    connect(outdoor_air_temp, min.u2) annotation(
      Line(points = {{-134, 30}, {-120, 30}, {-120, 40}, {-104, 40}}, color = {0, 0, 127}));
    connect(press_stp, dp_flow_fan.pres_stp) annotation(
      Line(points = {{-136, -36}, {-104, -36}, {-104, -14}, {-24, -14}}, color = {0, 0, 127}));
  connect(q_load, q_load_solver.q_load) annotation(
      Line(points = {{-136, -82}, {-17, -82}, {-17, -71}}, color = {0, 0, 127}));
  protected
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {40, 100}}), graphics = {Rectangle(origin = {-28, 0}, fillColor = {52, 101, 164}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-68, -20}, {68, 20}}), Rectangle(origin = {-30, -46}, lineColor = {255, 255, 255}, fillColor = {85, 87, 83}, fillPattern = FillPattern.Sphere, extent = {{-30, 30}, {30, -30}}), Polygon(origin = {-20, 60}, lineColor = {136, 138, 133}, fillColor = {186, 189, 182}, fillPattern = FillPattern.Sphere, points = {{-60, -40}, {-60, 40}, {40, 40}, {40, -40}, {20, -80}, {-8, -22}, {-40, -80}, {-60, -40}})}),
      uses(Modelica(version = "3.2.3"), Buildings(version = "7.0.0")));
  end ahu_v2;
  
  model ahu
    package Medium = Buildings.Media.Air;
    replaceable package MediumA = Buildings.Media.Air "Medium for air";
    parameter Real coil_air_V = 0.01 "air volume in coil m3";
    parameter Real coil_air_flow = 1 "nominal air mass flow in coil kg/s";
    parameter Real nom_air_dp = 150 "air pressure drop nominal";
    parameter Real nom_air_flow = 1 "nominal air mass flow in fan kg/s";
  
    Modelica.Blocks.Math.Min min annotation(
      Placement(visible = true, transformation(origin = {-92, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Sources.Boundary_pT sin(redeclare package Medium = MediumA, nPorts = 1) annotation(
      Placement(visible = true, transformation(extent = {{96, -46}, {76, -26}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput fan_power annotation(
      Placement(visible = true, transformation(origin = {108, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput air_flow annotation(
      Placement(visible = true, transformation(origin = {108, -64}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput chill_load annotation(
      Placement(visible = true, transformation(origin = {108, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium = MediumA, m_flow_nominal = 1) annotation(
      Placement(visible = true, transformation(origin = {-54, 6}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput suppl_temp annotation(
      Placement(visible = true, transformation(origin = {112, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Buildings.Fluid.Sources.Boundary_pT bou(redeclare package Medium = MediumA, nPorts = 1, use_T_in = true) annotation(
      Placement(visible = true, transformation(origin = {-82, 16}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    pres_stp.dp_flow_fan dp_flow_fan(redeclare package Medium = MediumA, dp_nominal = nom_air_dp, m_flow_nominal = nom_air_flow) annotation(
      Placement(visible = true, transformation(origin = {-18, -2}, extent = {{-12, 12}, {12, -12}}, rotation = 0)));
    pres_stp.simple_coil simple_coil(redeclare package Medium = MediumA, coil_air_V = coil_air_V, coil_air_flow = nom_air_flow) annotation(
      Placement(visible = true, transformation(origin = {18, 12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    pres_stp.q_load_solver q_load_solver annotation(
      Placement(visible = true, transformation(origin = {-12, -40}, extent = {{-10, 10}, {4, -10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput supl_stp annotation(
      Placement(visible = true, transformation(origin = {-134, 92}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-59, 119}, extent = {{-19, -19}, {19, 19}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealInput outdoor_air_temp annotation(
      Placement(visible = true, transformation(origin = {-134, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-119, 59}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput return_air_temp annotation(
      Placement(visible = true, transformation(origin = {-134, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-119, -1}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput press_stp annotation(
      Placement(visible = true, transformation(origin = {-136, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {7, 119}, extent = {{-19, -19}, {19, 19}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealInput q_load annotation(
      Placement(visible = true, transformation(origin = {-136, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-29, -119}, extent = {{-19, -19}, {19, 19}}, rotation = 90)));
  equation
    connect(dp_flow_fan.Flow, air_flow) annotation(
      Line(points = {{-5, -5}, {26.2, -5}, {26.2, -64}, {108, -64}}, color = {0, 0, 127}));
    connect(dp_flow_fan.Power, fan_power) annotation(
      Line(points = {{-5, -10}, {13.5, -10}, {13.5, -86}, {108, -86}}, color = {0, 0, 127}));
    connect(dp_flow_fan.port_b, simple_coil.port_a) annotation(
      Line(points = {{-6, 7.2}, {7, 7.2}, {7, 8.2}}, color = {0, 127, 255}));
    connect(min.y, bou.T_in) annotation(
      Line(points = {{-81, 46}, {-78.5, 46}, {-78.5, 28}, {-78, 28}}, color = {0, 0, 127}));
    connect(q_load_solver.air_flow, dp_flow_fan.Flow_stp) annotation(
      Line(points = {{-7, -40}, {0, -40}, {0, -14}, {-12, -14}}, color = {0, 0, 127}));
    connect(simple_coil.suppl_temp, suppl_temp) annotation(
      Line(points = {{30, 20}, {68, 20}, {68, 42}, {112, 42}}, color = {0, 0, 127}));
    connect(simple_coil.chill_load, chill_load) annotation(
      Line(points = {{30, 14}, {108, 14}}, color = {0, 0, 127}));
    connect(bou.ports[1], senTem.port_a) annotation(
      Line(points = {{-82, 6}, {-64, 6}}, color = {0, 127, 255}));
    connect(senTem.port_b, dp_flow_fan.port_a) annotation(
      Line(points = {{-44, 6}, {-30, 6}, {-30, 8}}, color = {0, 127, 255}));
    connect(simple_coil.port_b, sin.ports[1]) annotation(
      Line(points = {{30, 8}, {40, 8}, {40, -36}, {76, -36}}, color = {0, 127, 255}));
    connect(q_load_solver.supl_temp, supl_stp) annotation(
      Line(points = {{-23, -36}, {-38, -36}, {-38, 92}, {-134, 92}}, color = {0, 0, 127}));
    connect(supl_stp, simple_coil.supl_stp) annotation(
      Line(points = {{-134, 92}, {-38, 92}, {-38, 16}, {10, 16}}, color = {0, 0, 127}));
    connect(return_air_temp, min.u1) annotation(
      Line(points = {{-134, 64}, {-118, 64}, {-118, 52}, {-104, 52}}, color = {0, 0, 127}));
    connect(outdoor_air_temp, min.u2) annotation(
      Line(points = {{-134, 30}, {-120, 30}, {-120, 40}, {-104, 40}}, color = {0, 0, 127}));
    connect(press_stp, dp_flow_fan.pres_stp) annotation(
      Line(points = {{-136, -36}, {-104, -36}, {-104, -14}, {-24, -14}}, color = {0, 0, 127}));
    connect(q_load, q_load_solver.q_load) annotation(
      Line(points = {{-136, -82}, {-15, -82}, {-15, -51}}, color = {0, 0, 127}));
  connect(min.u1, q_load_solver.return_temp) annotation(
      Line(points = {{-104, 52}, {-46, 52}, {-46, -44}, {-22, -44}}, color = {0, 0, 127}));
  protected
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {40, 100}}), graphics = {Rectangle(origin = {-28, 0}, fillColor = {52, 101, 164}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-68, -20}, {68, 20}}), Rectangle(origin = {-30, -46}, lineColor = {255, 255, 255}, fillColor = {85, 87, 83}, fillPattern = FillPattern.Sphere, extent = {{-30, 30}, {30, -30}}), Polygon(origin = {-20, 60}, lineColor = {136, 138, 133}, fillColor = {186, 189, 182}, fillPattern = FillPattern.Sphere, points = {{-60, -40}, {-60, 40}, {40, 40}, {40, -40}, {20, -80}, {-8, -22}, {-40, -80}, {-60, -40}})}),
      uses(Modelica(version = "3.2.3"), Buildings(version = "7.0.0")));
  end ahu;

  model ahu_adapt
  
  parameter Real fan_a = 0;
  parameter Real fan_b = 1;
  parameter Real fan_c = 0;
  parameter Real chill_a = 0;
  parameter Real chill_b = 1;
  parameter Real chill_c = 0;
  
  parameter Real coil_air_V = 0.1;
  parameter Real nom_air_dp = 20; 
  parameter Real nom_air_flow = 1.8;
  parameter Real nom_coil_flow = 1.8;
  
  
  pres_stp.ahu ahu(coil_air_V = coil_air_V, coil_air_flow = nom_coil_flow, nom_air_dp = nom_air_dp, nom_air_flow = nom_air_flow)  annotation(
      Placement(visible = true, transformation(origin = {22.3321, -5.6679}, extent = {{-28.3321, -28.3321}, {11.3328, 28.3321}}, rotation = 0)));
  pres_stp.quadratick quadratick(a = fan_a, b = fan_b, c = fan_c)  annotation(
      Placement(visible = true, transformation(origin = {74.0875, 39.9356}, extent = {{-19.9356, -19.9356}, {7.97426, 19.9356}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput chill_load annotation(
      Placement(visible = true, transformation(origin = {110, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput fan_power annotation(
      Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Fahrenheit.ToKelvin toKelvin annotation(
      Placement(visible = true, transformation(origin = {-42, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Fahrenheit.ToKelvin toKelvin1 annotation(
      Placement(visible = true, transformation(origin = {-42, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Fahrenheit.ToKelvin toKelvin2 annotation(
      Placement(visible = true, transformation(origin = {-4, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  pres_stp.quadratick quadratick1(a = chill_a, b = chill_b, c = chill_c)  annotation(
      Placement(visible = true, transformation(origin = {76.0875, 3.9356}, extent = {{-19.9356, -19.9356}, {7.97426, 19.9356}}, rotation = 0)));
  Modelica.Blocks.Math.Gain wc_to_Pa(k = 248.84)  annotation(
      Placement(visible = true, transformation(origin = {22, 50}, extent = {{-8, -8}, {8, 8}}, rotation = -90)));
  Modelica.Blocks.Math.Gain q_flow_toC(k = 0.314632) annotation(
      Placement(visible = true, transformation(origin = {-37, -71}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput outdoor_air_temp annotation(
      Placement(visible = true, transformation(origin = {-100, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-119, 59}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput press_stp annotation(
      Placement(visible = true, transformation(origin = {-52, 92}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {7, 119}, extent = {{-19, -19}, {19, 19}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput return_air_temp annotation(
      Placement(visible = true, transformation(origin = {-100, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-119, -1}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput supl_stp annotation(
      Placement(visible = true, transformation(origin = {-52, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-59, 119}, extent = {{-19, -19}, {19, 19}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealInput q_load annotation(
      Placement(visible = true, transformation(origin = {-100, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-29, -119}, extent = {{-19, -19}, {19, 19}}, rotation = 90)));
  Modelica.Blocks.Math.Feedback feedback annotation(
      Placement(visible = true, transformation(origin = {30, -52}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
      Placement(visible = true, transformation(origin = {64, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput air_flow annotation(
      Placement(visible = true, transformation(origin = {110, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {70, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput wt_q_load annotation(
      Placement(visible = true, transformation(origin = {114, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = 3.14) annotation(
      Placement(visible = true, transformation(origin = {81, -73}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput p_q_load annotation(
      Placement(visible = true, transformation(origin = {110, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {70, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = 1004) annotation(
      Placement(visible = true, transformation(origin = {89, -37}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput suppl_temp_f annotation(
      Placement(visible = true, transformation(origin = {110, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {80, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Fahrenheit.FromKelvin fromKelvin annotation(
      Placement(visible = true, transformation(origin = {68, -94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  connect(ahu.fan_power, quadratick.u) annotation(
      Line(points = {{31, 20}, {35.9985, 20}, {35.9985, 40}, {47, 40}}, color = {0, 0, 127}));
    connect(quadratick.y, fan_power) annotation(
      Line(points = {{81, 40}, {110, 40}}, color = {0, 0, 127}));
  connect(toKelvin2.Kelvin, ahu.supl_stp) annotation(
      Line(points = {{-4, 39}, {-4, 30.5}, {0, 30.5}, {0, 28}}, color = {0, 0, 127}));
  connect(toKelvin1.Kelvin, ahu.outdoor_air_temp) annotation(
      Line(points = {{-30, 10}, {-17, 10}, {-17, 11}}, color = {0, 0, 127}));
  connect(ahu.return_air_temp, toKelvin.Kelvin) annotation(
      Line(points = {{-17, -6}, {-31, -6}}, color = {0, 0, 127}));
    connect(quadratick1.y, chill_load) annotation(
      Line(points = {{83, 4}, {110, 4}}, color = {0, 0, 127}));
  connect(quadratick1.u, ahu.chill_load) annotation(
      Line(points = {{49, 4}, {31, 4}}, color = {0, 0, 127}));
  connect(wc_to_Pa.y, ahu.press_stp) annotation(
      Line(points = {{22, 41}, {22, 30.5}, {19, 30.5}, {19, 28}}, color = {0, 0, 127}));
  connect(q_flow_toC.y, ahu.q_load) annotation(
      Line(points = {{-29, -71}, {9, -71}, {9, -39}}, color = {0, 0, 127}));
    connect(supl_stp, toKelvin2.Fahrenheit) annotation(
      Line(points = {{-52, 74}, {-4, 74}, {-4, 62}}, color = {0, 0, 127}));
    connect(press_stp, wc_to_Pa.u) annotation(
      Line(points = {{-52, 92}, {22, 92}, {22, 60}}, color = {0, 0, 127}));
    connect(outdoor_air_temp, toKelvin1.Fahrenheit) annotation(
      Line(points = {{-100, 10}, {-54, 10}}, color = {0, 0, 127}));
    connect(return_air_temp, toKelvin.Fahrenheit) annotation(
      Line(points = {{-100, -6}, {-54, -6}}, color = {0, 0, 127}));
    connect(q_load, q_flow_toC.u) annotation(
      Line(points = {{-100, -70}, {-73, -70}, {-73, -71}, {-45, -71}}, color = {0, 0, 127}));
    connect(feedback.y, product.u2) annotation(
      Line(points = {{40, -52}, {44, -52}, {44, -44}, {52, -44}}, color = {0, 0, 127}));
  connect(ahu.air_flow, air_flow) annotation(
      Line(points = {{31, -17}, {70, -17}, {70, -18}, {110, -18}}, color = {0, 0, 127}));
  connect(ahu.suppl_temp, feedback.u2) annotation(
      Line(points = {{31, -28}, {31, -36}, {30, -36}, {30, -44}}, color = {0, 0, 127}));
  connect(ahu.air_flow, product.u1) annotation(
      Line(points = {{31, -17}, {44, -17}, {44, -32}, {52, -32}}, color = {0, 0, 127}));
    connect(feedback.u1, toKelvin.Kelvin) annotation(
      Line(points = {{22, -52}, {-30, -52}, {-30, -6}}, color = {0, 0, 127}));
    connect(gain.y, p_q_load) annotation(
      Line(points = {{88, -72}, {110, -72}}, color = {0, 0, 127}));
  connect(product.y, gain1.u) annotation(
      Line(points = {{76, -38}, {79, -38}, {79, -36}, {80, -36}}, color = {0, 0, 127}));
  connect(gain1.y, wt_q_load) annotation(
      Line(points = {{97, -37}, {114, -37}, {114, -38}}, color = {0, 0, 127}));
  connect(gain1.y, gain.u) annotation(
      Line(points = {{96, -36}, {98, -36}, {98, -54}, {64, -54}, {64, -72}, {72, -72}}, color = {0, 0, 127}));
  connect(fromKelvin.Fahrenheit, suppl_temp_f) annotation(
      Line(points = {{80, -94}, {110, -94}, {110, -92}}, color = {0, 0, 127}));
  connect(fromKelvin.Kelvin, ahu.suppl_temp) annotation(
      Line(points = {{56, -94}, {38, -94}, {38, -28}, {31, -28}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {40, 100}})));
  end ahu_adapt;

  model quadratick
  
  parameter Real a = 1;
  parameter Real b = 1;
  parameter Real c = 1;
  
  Modelica.Blocks.Interfaces.RealInput u annotation(
      Placement(visible = true, transformation(origin = {-116, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-119, -1}, extent = {{-19, -19}, {19, 19}}, rotation = 0)));
  Modelica.Blocks.Math.Gain const_a(k = a)  annotation(
      Placement(visible = true, transformation(origin = {-40, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain const_b(k = b)  annotation(
      Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
      Placement(visible = true, transformation(origin = {-72, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const_c(k = c) annotation(
      Placement(visible = true, transformation(origin = {-70, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add3 add3 annotation(
      Placement(visible = true, transformation(origin = {28, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y annotation(
      Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(u, const_b.u) annotation(
      Line(points = {{-116, 0}, {-82, 0}}, color = {0, 0, 127}));
    connect(product.y, const_a.u) annotation(
      Line(points = {{-61, 60}, {-53, 60}}, color = {0, 0, 127}));
    connect(u, product.u1) annotation(
      Line(points = {{-116, 0}, {-94, 0}, {-94, 66}, {-84, 66}}, color = {0, 0, 127}));
    connect(u, product.u2) annotation(
      Line(points = {{-116, 0}, {-94, 0}, {-94, 54}, {-84, 54}}, color = {0, 0, 127}));
  connect(const_a.y, add3.u1) annotation(
      Line(points = {{-28, 60}, {-22, 60}, {-22, 8}, {16, 8}}, color = {0, 0, 127}));
  connect(const_b.y, add3.u2) annotation(
      Line(points = {{-58, 0}, {16, 0}}, color = {0, 0, 127}));
  connect(const_c.y, add3.u3) annotation(
      Line(points = {{-58, -60}, {-40, -60}, {-40, -8}, {16, -8}}, color = {0, 0, 127}));
  connect(add3.y, y) annotation(
      Line(points = {{40, 0}, {110, 0}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {40, 100}}), graphics = {Rectangle(origin = {-31.82, -0.21}, fillColor = {211, 215, 207}, fillPattern = FillPattern.Solid, extent = {{-60.18, 60.21}, {60.18, -60.21}}), Line(origin = {-34.61, -2.84}, points = {{-39.673, 43.3531}, {-39.673, -38.6469}, {44.327, -38.6469}, {32.327, -42.6469}, {32.327, -34.6469}, {42.327, -38.6469}, {-39.673, -38.6469}, {-39.673, 43.3531}, {-43.673, 27.3531}, {-35.673, 27.3531}, {-39.673, 43.3531}}, thickness = 0.75), Line(origin = {-24.41, 5.87}, points = {{-41.935, -27.1325}, {-27.935, -37.1325}, {-9.93497, -37.1325}, {8.06503, -25.1325}, {32.065, 4.86754}, {40.065, 30.8675}, {42.065, 36.8675}}, color = {204, 0, 0}, thickness = 1, smooth = Smooth.Bezier)}));
  end quadratick;
  
  model ahu_adapt_tests
  
  parameter Real fan_a = 0;
  parameter Real fan_b = 1;
  parameter Real fan_c = 0;
  parameter Real chill_a = 0;
  parameter Real chill_b = 1;
  parameter Real chill_c = 0;
  
  parameter Real coil_air_V = 0.1;
  parameter Real nom_air_dp = 20; 
  parameter Real nom_air_flow = 1.8;
  pres_stp.ahu ahu(coil_air_V = coil_air_V, coil_air_flow = 0.1, nom_air_dp = nom_air_dp, nom_air_flow = nom_air_flow)  annotation(
      Placement(visible = true, transformation(origin = {22.3321, -5.6679}, extent = {{-28.3321, -28.3321}, {11.3328, 28.3321}}, rotation = 0)));
  pres_stp.quadratick quadratick(a = fan_a, b = fan_b, c = fan_c)  annotation(
      Placement(visible = true, transformation(origin = {74.0875, 39.9356}, extent = {{-19.9356, -19.9356}, {7.97426, 19.9356}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput chill_load annotation(
      Placement(visible = true, transformation(origin = {110, 4}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput fan_power annotation(
      Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 92}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Fahrenheit.ToKelvin toKelvin annotation(
      Placement(visible = true, transformation(origin = {-42, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Fahrenheit.ToKelvin toKelvin1 annotation(
      Placement(visible = true, transformation(origin = {-42, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Fahrenheit.ToKelvin toKelvin2 annotation(
      Placement(visible = true, transformation(origin = {-4, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  pres_stp.quadratick quadratick1(a = chill_a, b = chill_b, c = chill_c)  annotation(
      Placement(visible = true, transformation(origin = {76.0875, 3.9356}, extent = {{-19.9356, -19.9356}, {7.97426, 19.9356}}, rotation = 0)));
  Modelica.Blocks.Math.Gain wc_to_Pa(k = 248.84)  annotation(
      Placement(visible = true, transformation(origin = {22, 50}, extent = {{-8, -8}, {8, 8}}, rotation = -90)));
  Modelica.Blocks.Math.Gain q_flow_toC(k = 0.31812) annotation(
      Placement(visible = true, transformation(origin = {-37, -71}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const_c(k = 79991) annotation(
      Placement(visible = true, transformation(origin = {-92, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 77.55) annotation(
      Placement(visible = true, transformation(origin = {-112, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = 1.3)  annotation(
      Placement(visible = true, transformation(origin = {-62, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant3(k = 69.52) annotation(
      Placement(visible = true, transformation(origin = {-100, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 83.26) annotation(
      Placement(visible = true, transformation(origin = {-116, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback annotation(
      Placement(visible = true, transformation(origin = {30, -52}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
  Modelica.Blocks.Math.Product product annotation(
      Placement(visible = true, transformation(origin = {64, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = 1004) annotation(
      Placement(visible = true, transformation(origin = {89, -37}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput real_q_load annotation(
      Placement(visible = true, transformation(origin = {116, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput air_flow annotation(
      Placement(visible = true, transformation(origin = {110, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {70, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = 3.14) annotation(
      Placement(visible = true, transformation(origin = {81, -73}, extent = {{-7, -7}, {7, 7}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput p_q_load annotation(
      Placement(visible = true, transformation(origin = {110, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {70, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Fahrenheit.FromKelvin fromKelvin annotation(
      Placement(visible = true, transformation(origin = {68, -94}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput suppl_temp_f annotation(
      Placement(visible = true, transformation(origin = {110, -92}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {80, 64}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(ahu.fan_power, quadratick.u) annotation(
      Line(points = {{30.9985, 20.3976}, {35.9985, 20.3976}, {35.9985, 40}, {47, 40}}, color = {0, 0, 127}));
    connect(quadratick.y, fan_power) annotation(
      Line(points = {{81, 40}, {110, 40}}, color = {0, 0, 127}));
    connect(toKelvin2.Kelvin, ahu.supl_stp) annotation(
      Line(points = {{-4, 39}, {-4, 30.5}, {0, 30.5}, {0, 28}}, color = {0, 0, 127}));
    connect(toKelvin1.Kelvin, ahu.outdoor_air_temp) annotation(
      Line(points = {{-30, 10}, {-16, 10}, {-16, 12}}, color = {0, 0, 127}));
    connect(ahu.return_air_temp, toKelvin.Kelvin) annotation(
      Line(points = {{-16, -6}, {-31, -6}}, color = {0, 0, 127}));
    connect(quadratick1.y, chill_load) annotation(
      Line(points = {{83, 4}, {110, 4}}, color = {0, 0, 127}));
    connect(quadratick1.u, ahu.chill_load) annotation(
      Line(points = {{49, 4}, {30, 4}}, color = {0, 0, 127}));
    connect(wc_to_Pa.y, ahu.press_stp) annotation(
      Line(points = {{22, 41}, {22, 30.5}, {18, 30.5}, {18, 28}}, color = {0, 0, 127}));
    connect(q_flow_toC.y, ahu.q_load) annotation(
      Line(points = {{-29, -71}, {8, -71}, {8, -40}}, color = {0, 0, 127}));
    connect(const_c.y, q_flow_toC.u) annotation(
      Line(points = {{-80, -72}, {-46, -72}, {-46, -70}}, color = {0, 0, 127}));
    connect(constant1.y, toKelvin.Fahrenheit) annotation(
      Line(points = {{-100, -12}, {-54, -12}, {-54, -6}}, color = {0, 0, 127}));
    connect(toKelvin1.Fahrenheit, const.y) annotation(
      Line(points = {{-54, 10}, {-104, 10}, {-104, 20}}, color = {0, 0, 127}));
    connect(constant2.y, wc_to_Pa.u) annotation(
      Line(points = {{-51, 68}, {23.5, 68}, {23.5, 60}, {22, 60}}, color = {0, 0, 127}));
    connect(constant3.y, toKelvin2.Fahrenheit) annotation(
      Line(points = {{-89, 90}, {-4, 90}, {-4, 62}}, color = {0, 0, 127}));
    connect(ahu.suppl_temp, feedback.u2) annotation(
      Line(points = {{30, -28}, {30, -44}}, color = {0, 0, 127}));
    connect(feedback.y, product.u2) annotation(
      Line(points = {{40, -52}, {44, -52}, {44, -44}, {52, -44}}, color = {0, 0, 127}));
    connect(ahu.air_flow, product.u1) annotation(
      Line(points = {{30, -18}, {42, -18}, {42, -32}, {52, -32}}, color = {0, 0, 127}));
    connect(product.y, gain.u) annotation(
      Line(points = {{76, -38}, {78, -38}, {78, -36}, {80, -36}}, color = {0, 0, 127}));
    connect(feedback.u1, toKelvin.Kelvin) annotation(
      Line(points = {{22, -52}, {-30, -52}, {-30, -6}}, color = {0, 0, 127}));
    connect(ahu.air_flow, air_flow) annotation(
      Line(points = {{30, -18}, {110, -18}}, color = {0, 0, 127}));
    connect(gain.y, real_q_load) annotation(
      Line(points = {{96, -36}, {116, -36}}, color = {0, 0, 127}));
    connect(gain1.y, p_q_load) annotation(
      Line(points = {{88, -72}, {110, -72}}, color = {0, 0, 127}));
    connect(gain1.u, gain.y) annotation(
      Line(points = {{72, -72}, {60, -72}, {60, -54}, {96, -54}, {96, -36}}, color = {0, 0, 127}));
    connect(fromKelvin.Fahrenheit, suppl_temp_f) annotation(
      Line(points = {{80, -94}, {110, -94}, {110, -92}}, color = {0, 0, 127}));
  connect(ahu.suppl_temp, fromKelvin.Kelvin) annotation(
      Line(points = {{30, -28}, {40, -28}, {40, -94}, {56, -94}}, color = {0, 0, 127}));
    annotation(
      Icon(coordinateSystem(extent = {{-100, -100}, {40, 100}})));
  end ahu_adapt_tests;
  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {40, 100}})),
  uses(Modelica(version = "3.2.3"), Buildings(version = "7.0.0")));
end pres_stp;