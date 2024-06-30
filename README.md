# 3d_Curved_Beam
import openseespy.opensees as ops
import math
import matplotlib.pyplot as plt

# Defining the material properties for concrete and steel
E_concrete = 26600.0  # Young's modulus in MPa
nu_concrete = 0.2     # Poisson's ratio concrete that is used
rho_concrete = 2.5e-9  # Density in tonnes/mm^3 (2500 kg/m^3)
fpc = 30.0  # Compressive strength of concrete in MPa
crushing_strength = -0.2 * fpc
#properties of gradde 60 steel in MPA
E_steel = 200000.0  # Young's modulus for steel in MPa
fy_steel = 413.60    # Yield strength of steel in MPa

# Define beam section dimensions and reinforcement
beam_width = 800.0   # Beam width in mm
beam_height = 800.0  # Beam height in mm
cover = 40.0         # Cover to reinforcement in mm
num_bars_top = 2     # Number of reinforcement bars at the top
num_bars_bottom = 2  # Number of reinforcement bars at the bottom
bar_diameter = 16.0  # Diameter of reinforcement bars in mm

# Calculate areas of bar
bar_area = math.pi * (bar_diameter / 2)**2

# Inclusion of Torsional properties in 3 dimensional study
G_concrete = E_concrete / (2 * (1 + nu_concrete))  # Shear modulus in MPa

# Torsional properties using the approximate formula used for the rectangular

b = beam_width
h = beam_height

J = (b * h**3 / 3) * ((16 / 3) - (3.36 * b / h) * (1 - (b**4 / (12 * h**4))))

# Initalize the OpenSees model in 3 Dimensional
ops.wipe()

ops.model('basic', '-ndm', 3, '-ndf', 6)

# Define coordinate transformation linear from local to global axes with a vector to define local z-axis

ops.geomTransf('Linear', 1, 0, 0, 1)

# Define concrete and steel materials

ops.uniaxialMaterial('Concrete01', 1, -fpc, -0.002,crushing_strength, -0.006)
ops.uniaxialMaterial('Steel01', 2, fy_steel, E_steel, 0.01)
ops.uniaxialMaterial('Elastic', 3, G_concrete)  # Torsional material

# Define the section with fibers

ops.section('Fiber', 1, '-GJ', J)
ops.patch('rect', 1, 10, 10, -beam_width/2, -beam_height/2, beam_width/2, beam_height/2)  # Concrete core
ops.layer('straight', 2, num_bars_top, bar_area, -beam_width/2 + cover, beam_height/2 - cover, beam_width/2 - cover, beam_height/2 - cover)  # Top bars
ops.layer('straight', 2, num_bars_bottom, bar_area, -beam_width/2 + cover, -beam_height/2 + cover, beam_width/2 - cover, -beam_height/2 + cover)  # Bottom bars

# Geometry parameters

radius = 10.0  # Radius of the circle in mm

# Define the portion of the circle either we want the quarter, semi, or full circle

circle_portion = 'quarter'  # Options: 'quarter', 'semi', 'full'

# Determine the angle range based on the circle portion
if circle_portion == 'quarter':
    angle_range = math.pi / 2
elif circle_portion == 'semi':
    angle_range = math.pi
elif circle_portion == 'full':
    angle_range = 2 * math.pi
else:
    raise ValueError("Invalid circle portion. Choose 'quarter', 'semi', or 'full'.")

# Define nodes for the circle portion
num_points = 50  # Number of points required for the curve
for i in range(num_points + 1):
    theta = angle_range * (i / num_points)
    x = radius * math.cos(theta)
    y = 0.0  # Y-coordinate set to 0.0
    z = radius * math.sin(theta)  # Z-coordinate (assuming the circle lies in the XZ-plane)
    ops.node(i, x, y, z)

# Fix the first node 0 to all 6 degree of freedom
ops.fix(0, 1, 1, 1, 1, 1, 1)

# Define 3D beam elements with fiber section between nodes
ops.beamIntegration('Lobatto', 1, 1, 3)  # Lobatto integration for the fiber section and for curved shapes effecient model
for i in range(num_points):
    ops.element('forceBeamColumn', i + 1, i, i + 1, 1, 1)

# Define gravity loads (assuming a vertical load in the negative Z direction)
fx = 0.0
fy = 0.0
fz = -0.1
Mx = 0.0
My = 0.1
Mz = 0.0

# Apply gravity loads to all nodes except the fixed node
ops.timeSeries('Linear', 1)
ops.pattern('Plain', 1, 1)
for i in range(1, num_points + 1):
    ops.load(i, fx , fy , fz, Mx, My , Mz )

# Analysis techniques
ops.system('BandGeneral')
ops.numberer('Plain')
ops.constraints('Plain')
ops.test('NormUnbalance', 1e-6, 25)
ops.algorithm('Newton')
ops.integrator('LoadControl', 0.1)  # Load increment
ops.analysis('Static')

# Run the analysis incrementally loads
num_steps = 10
for step in range(num_steps):
    if ops.analyze(1) != 0:
        print(f"Analysis failed at step {step + 1}")
        break

# Storing the coordinates of original and deflected shape
original_x = []
original_y = []
original_z = []
deflected_x = []
deflected_y = []
deflected_z = []

# Retrieve and store node displacements
displacements = []
for i in range(num_points + 1):
    disp = ops.nodeDisp(i)
    displacements.append(disp)
    print(f"Node {i} displacement: {disp}")

    # Store original coordinates
    original_x.append(ops.nodeCoord(i)[0])
    original_y.append(ops.nodeCoord(i)[1])
    original_z.append(ops.nodeCoord(i)[2])

    # Store deflected coordinates
    deflected_x.append(ops.nodeCoord(i)[0] + disp[0])
    deflected_y.append(ops.nodeCoord(i)[1] + disp[1])
    deflected_z.append(ops.nodeCoord(i)[2] + disp[2])

# Extract z-direction displacements for 2D plot
z_displacements = [disp[2] for disp in displacements]

# Plotting the original and deflected shape
fig = plt.figure(figsize=(14, 6))

# 3D Plot
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot(original_x, original_y, original_z, 'o-', label=f'Original {circle_portion.capitalize()} Circle')
ax1.plot(deflected_x, deflected_y, deflected_z, 'x--', label='Deflected Shape')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.set_title('3D View')
ax1.legend()
ax1.grid(True)

# Rotate the view angle
ax1.view_init(elev=30, azim=45)

# 2D Plot with exaggerated deflection
exaggeration_factor = 10
ax2 = fig.add_subplot(122)
ax2.plot(original_x, original_z, 'o-', label='Original Shape')
ax2.plot(original_x, [z + exaggeration_factor * dz for z, dz in zip(original_z, z_displacements)], 'x--', label='Deflected Shape (exaggerated)')
ax2.set_xlabel('X')
ax2.set_ylabel('Z')
ax2.set_title('2D View with Exaggerated Deflection')
ax2.legend()
ax2.grid(True)

plt.suptitle(f'Deflection of {circle_portion.capitalize()} Circle')
plt.show()
