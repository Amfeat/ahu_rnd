model ahu
  replaceable package MediumA = Buildings.Media.Air "Medium for air";

  Modelica.Blocks.Interfaces.RealInput st_pres_stp annotation(
    Placement(visible = true, transformation(origin = {-116, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-120, 30}, {-100, 50}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput supl_stp annotation(
    Placement(visible = true, transformation(origin = {-114, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{-120, -52}, {-100, -32}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput oat annotation(
    Placement(visible = true, transformation(origin = {-112, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, -110},extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput return_temp annotation(
    Placement(visible = true, transformation(origin = {-118, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-60, -110},extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Math.Min min annotation(
    Placement(visible = true, transformation(origin = {-88, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.LimPID pid(limitsAtInit = true, yMax = 1000, yMin = 0) annotation(
    Placement(visible = true, transformation(origin = {-78, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Q_load annotation(
    Placement(visible = true, transformation(origin = {0, 110}, extent = {{-10, -10}, {10, 10}}, rotation = -90), iconTransformation(origin = {-30, 112},extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Buildings.Fluid.Sources.MassFlowSource_T ventilationOut(redeclare package Medium = MediumA, nPorts = 1, use_T_in = true, use_m_flow_in = true) annotation(
    Placement(visible = true, transformation(extent = {{-46, -40}, {-26, -20}}, rotation = 0)));
  Buildings.Fluid.Sources.Boundary_pT sin(redeclare package Medium = MediumA, nPorts = 1) annotation(
    Placement(visible = true, transformation(extent = {{80, -40}, {60, -20}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback delta_temp annotation(
    Placement(visible = true, transformation(origin = {-20, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = 1005)  annotation(
    Placement(visible = true, transformation(origin = {-42, 78}, extent = {{6, -6}, {-6, 6}}, rotation = 0)));
  Modelica.Blocks.Math.Division division annotation(
    Placement(visible = true, transformation(origin = {-74, 84}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
Modelica.Blocks.Interfaces.RealOutput fan_power annotation(
    Placement(visible = true, transformation(origin = {110, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Blocks.Interfaces.RealOutput air_flow annotation(
    Placement(visible = true, transformation(origin = {110, -72}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Blocks.Interfaces.RealOutput chill_load annotation(
    Placement(visible = true, transformation(origin = {160, 68}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  pres_stp.fan_dp fan_dp annotation(
    Placement(visible = true, transformation(origin = {-32, -66}, extent = {{-12, -12}, {12, 12}}, rotation = 0)));
  Buildings.Fluid.MixingVolumes.MixingVolume vol(redeclare package Medium = MediumA, V = 1, m_flow_nominal = 1, nPorts = 2) annotation(
    Placement(visible = true, transformation(origin = {1, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHea(alpha = 0) annotation(
    Placement(visible = true, transformation(extent = {{-32, 0}, {-12, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = -1) annotation(
    Placement(visible = true, transformation(origin = {-46, 10}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium = MediumA, m_flow_nominal = 1) annotation(
    Placement(visible = true, transformation(origin = {32, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(return_temp, min.u1) annotation(
    Line(points = {{-118, 20}, {-100, 20}, {-100, 8}}, color = {0, 0, 127}));
  connect(oat, min.u2) annotation(
    Line(points = {{-112, -20}, {-100, -20}, {-100, -4}}, color = {0, 0, 127}));
  connect(supl_stp, pid.u_s) annotation(
    Line(points = {{-114, 50}, {-90, 50}}, color = {0, 0, 127}));
  connect(ventilationOut.T_in, min.y) annotation(
    Line(points = {{-48, -26}, {-65, -26}, {-65, 2}, {-76, 2}}, color = {0, 0, 127}));
  connect(return_temp, delta_temp.u1) annotation(
    Line(points = {{-118, 20}, {-60, 20}, {-60, 48}, {-28, 48}}, color = {0, 0, 127}));
  connect(delta_temp.y, gain.u) annotation(
    Line(points = {{-11, 48}, {-12, 48}, {-12, 78}, {-35, 78}}, color = {0, 0, 127}));
  connect(gain.y, division.u2) annotation(
    Line(points = {{-49, 78}, {-62, 78}}, color = {0, 0, 127}));
  connect(Q_load, division.u1) annotation(
    Line(points = {{0, 110}, {0, 90}, {-62, 90}}, color = {0, 0, 127}));
  connect(fan_dp.pres_stp, st_pres_stp) annotation(
    Line(points = {{-46, -74}, {-116, -74}}, color = {0, 0, 127}));
  connect(fan_dp.Power, fan_power) annotation(
    Line(points = {{-20, -58}, {110, -58}}, color = {0, 0, 127}));
  connect(fan_dp.Flow, air_flow) annotation(
    Line(points = {{-18, -72}, {110, -72}}, color = {0, 0, 127}));
  connect(fan_dp.Flow, ventilationOut.m_flow_in) annotation(
    Line(points = {{-18, -72}, {-6, -72}, {-6, -50}, {-76, -50}, {-76, -22}, {-48, -22}}, color = {0, 0, 127}));
  connect(preHea.port, vol.heatPort) annotation(
    Line(points = {{-12, 10}, {-9, 10}, {-9, -20}}, color = {191, 0, 0}));
  connect(ventilationOut.ports[1], vol.ports[1]) annotation(
    Line(points = {{-26, -30}, {1, -30}}, color = {0, 127, 255}));
  connect(pid.y, chill_load) annotation(
    Line(points = {{-67, 50}, {-62.5, 50}, {-62.5, 68}, {160, 68}}, color = {0, 0, 127}));
  connect(gain1.y, preHea.Q_flow) annotation(
    Line(points = {{-39.4, 10}, {-32.4, 10}}, color = {0, 0, 127}));
  connect(gain1.u, pid.y) annotation(
    Line(points = {{-53, 10}, {-62.5, 10}, {-62.5, 50}, {-66, 50}}, color = {0, 0, 127}));
  connect(senTem.port_a, vol.ports[2]) annotation(
    Line(points = {{22, -10}, {14, -10}, {14, -30}, {2, -30}}, color = {0, 127, 255}));
  connect(senTem.port_b, sin.ports[1]) annotation(
    Line(points = {{42, -10}, {48, -10}, {48, -30}, {60, -30}}, color = {0, 127, 255}));
  connect(senTem.T, delta_temp.u2) annotation(
    Line(points = {{32, 2}, {32, 40}, {-20, 40}}, color = {0, 0, 127}));
  connect(senTem.T, pid.u_m) annotation(
    Line(points = {{32, 2}, {22, 2}, {22, 30}, {-78, 30}, {-78, 38}}, color = {0, 0, 127}));
  connect(fan_dp.Flow_stp, division.y) annotation(
    Line(points = {{-44, -60}, {-152, -60}, {-152, 86}, {-84, 86}, {-84, 84}}, color = {0, 0, 127}));
protected
  annotation(
    Icon(coordinateSystem(extent = {{-100, -100}, {40, 100}}), graphics = {Rectangle(origin = {-28, 0}, fillColor = {52, 101, 164}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-68, -20}, {68, 20}}), Rectangle(origin = {-30, -46}, lineColor = {255, 255, 255}, fillColor = {85, 87, 83}, fillPattern = FillPattern.Sphere, extent = {{-30, 30}, {30, -30}}), Polygon(origin = {-20, 60}, lineColor = {136, 138, 133}, fillColor = {186, 189, 182}, fillPattern = FillPattern.Sphere, points = {{-60, -40}, {-60, 40}, {40, 40}, {40, -40}, {20, -80}, {-8, -22}, {-40, -80}, {-60, -40}})}),
    uses(Modelica(version = "3.2.3"), Buildings(version = "7.0.0")));
end ahu;