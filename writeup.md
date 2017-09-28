# Pick & Place Project

## Kinematics Analysis

To perform the kinematics analysis of the Kuka 210 we first sketch it in its zero configuration. We then label all the joints from 1 to N, starting at the base. We then add a line to indicate the axis the joint rotates about. At this point we can make a mental note of which joints are parallel or coincident as this will be relevant when we are filling out values in our DH table later on. We then label all the links from 0 to N. If N is the number of joints then we have N + 1 links in total.
We then define the common normals and reference frame origins. The Z axes is the axes we marked earlier, and we add the _X_ axes as a normal to _Zi-1_ and _Zi_. The reference frame origins are defined as the point of intersection between the _Zi_ axis and _Xi_ axis where _i_ indicates the link number. These are annotated as _Oi_. We add one final frame for the gripper (this the one we care about when we are trying to grab stuff after all). It is attached to link 6. Next we add the link lengths (_Ai_) and link offsets (_Di_). The link lengths are the distance between the _Z_ axes, measured along the _X_ axes. The link offsets are the distance between the _X_ axes, measured along the _Z_ axes (the actual values for these can be gathered from the urdf file). We also add the twist angles (_αi_) defined as the angle between _Zi-1_ and _Zi_ measured about the _Xi-1_ axes (using the right hand rule). _θi_ is the measurement of the angle between _Xi-1_ and _Xi_ measured about the _Zi_ axes (note the -90deg offset for i=2 between X1 and X2). We also need to convert the gripper frame from the URDF to be the correct orientation for our DH parameters. I'll make a note of where this happens in the discussion of the python code in the following section.

[kinematics_analysis]: ./kinematics_analysis.jpg
![kinematics analysis][kinematics_analysis]

### Forward Kinematics

#### i = 1

```xml
<joint name="fixed_base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
<joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
</joint>
<joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
</joint>
```

`α = 0` as z<sub>0</sub> and z<sub>1</sub> are parallel

`a = 0` as z<sub>0</sub> and z<sub>1</sub> are coincident

`d` = X<sub>0</sub> -> X<sub>1</sub> = `0.42 + 0.33 = 0.75`

Links                     | α<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | θ<sub>i</sub>
---                       | ---             | ---             | ---           | ---
<sup>0</sup>T<sub>1</sub> | 0               | 0               | 0.75          | q<sub>1</sub>

#### i = 2

```xml
<joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
</joint>
<joint name="joint_2" type="revolute">
    <origin xyz="0.35 0 0.42" rpy="0 0 0"/>
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-45*deg}" upper="${85*deg}" effort="300" velocity="${115*deg}"/>
</joint>
```

`α = -pi/2` as Z<sub>1</sub> and Z<sub>2</sub> are perpendicular (signed as per right hand rule)

`a` = Z<sub>x2</sub> - Z<sub>x1</sub> = `0.35 - 0 = 0.35`

`d = 0`

Links                     | α<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | θ<sub>i</sub>
---                       | ---             | ---             | ---           | ---
<sup>0</sup>T<sub>1</sub> | 0               | 0               | 0.75          | q<sub>1</sub>
<sup>1</sup>T<sub>2</sub> | -pi/2           | 0.35            | 0             | -pi/2 + q<sub>2</sub>

#### i = 3

```xml
<joint name="joint_3" type="revolute">
    <origin xyz="0 0 1.25" rpy="0 0 0"/>
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-210*deg}" upper="${(155-90)*deg}" effort="300" velocity="${112*deg}"/>
</joint>
```

`α = 0` as z<sub>2</sub> and z<sub>3</sub> are parallel

`a = 0.35`

`d = 0` as the common normal intersects Ž<sub>i</sub> at the origin of frame i

Links                     | α<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | θ<sub>i</sub>
---                       | ---             | ---             | ---           | ---
<sup>0</sup>T<sub>1</sub> | 0               | 0               | 0.75          | q<sub>1</sub>
<sup>1</sup>T<sub>2</sub> | -pi/2           | 0.35            | 0             | -pi/2 + q<sub>2</sub>
<sup>2</sup>T<sub>3</sub> | 0               | 1.25            | 0             | q<sub>3</sub>

#### i = 4

```xml
<joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
</joint>
<joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
</joint>
```

`α = -pi/2` as Z<sub>1</sub> and Z<sub>2</sub> are perpendicular (signed as per right hand rule)

`a = 0.054`

`d` = X<sub>3</sub> -> X<sub>4</sub> = `0.54 + 0.96 = 1.50`

Links                     | α<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | θ<sub>i</sub>
---                       | ---             | ---             | ---           | ---
<sup>0</sup>T<sub>1</sub> | 0               | 0               | 0.75          | q<sub>1</sub>
<sup>1</sup>T<sub>2</sub> | -pi/2           | 0.35            | 0             | -pi/2 + q<sub>2</sub>
<sup>2</sup>T<sub>3</sub> | 0               | 1.25            | 0             | q<sub>3</sub>
<sup>3</sup>T<sub>4</sub> | -pi/2           | -0.054          | 1.50          | q<sub>4</sub>

#### i = 5

```xml
<joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
</joint>
<joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
</joint>
```

`α = pi/2` as Z<sub>1</sub> and Z<sub>2</sub> are perpendicular (signed as per right hand rule)

`a = 0` as z<sub>0</sub> and z<sub>1</sub> are coincident

`d = 0` as the common normal intersects Ž<sub>i</sub> at the origin of frame i

Links                     | α<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | θ<sub>i</sub>
---                       | ---             | ---             | ---           | ---
<sup>0</sup>T<sub>1</sub> | 0               | 0               | 0.75          | q<sub>1</sub>
<sup>1</sup>T<sub>2</sub> | -pi/2           | 0.35            | 0             | -pi/2 + q<sub>2</sub>
<sup>2</sup>T<sub>3</sub> | 0               | 1.25            | 0             | q<sub>3</sub>
<sup>3</sup>T<sub>4</sub> | -pi/2           | -0.054          | 1.50          | q<sub>4</sub>
<sup>4</sup>T<sub>5</sub> | pi/2            | 0               | 0             | q<sub>5</sub>

#### i = 6

```xml
<joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
</joint>
```

`α = pi/2` as Z<sub>1</sub> and Z<sub>2</sub> are perpendicular (signed as per right hand rule)

`a = 0` as z<sub>0</sub> and z<sub>1</sub> are coincident

`d = 0` as the common normal intersects Ž<sub>i</sub> at the origin of frame i

Links                     | α<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | θ<sub>i</sub>
---                       | ---             | ---             | ---           | ---
<sup>0</sup>T<sub>1</sub> | 0               | 0               | 0.75          | q<sub>1</sub>
<sup>1</sup>T<sub>2</sub> | -pi/2           | 0.35            | 0             | -pi/2 + q<sub>2</sub>
<sup>2</sup>T<sub>3</sub> | 0               | 1.25            | 0             | q<sub>3</sub>
<sup>3</sup>T<sub>4</sub> | -pi/2           | -0.054          | 1.50          | q<sub>4</sub>
<sup>4</sup>T<sub>5</sub> | pi/2            | 0               | 0             | q<sub>5</sub>
<sup>5</sup>T<sub>6</sub> | -pi/2           | 0               | 0             | q<sub>6</sub>

#### i = 7 (or EE or however you want to refer to the gripper)
```xml
<joint name="joint_4" type="revolute">
    <origin xyz="0.96 0 -0.054" rpy="0 0 0"/>
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${179*deg}"/>
</joint>
<joint name="joint_5" type="revolute">
    <origin xyz="0.54 0 0" rpy="0 0 0"/>
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-125*deg}" upper="${125*deg}" effort="300" velocity="${172*deg}"/>
</joint>
<joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
</joint>
<joint name="gripper_joint" type="fixed">
    <parent link="link_6"/>
    <child link="gripper_link"/>
    <origin xyz="0.0375 0 0" rpy="0 0 0"/>
</joint>
```

`α = 0` as z<sub>0</sub> and z<sub>1</sub> are parallel

`a = 0` as z<sub>0</sub> and z<sub>1</sub> are coincident

`d = 0.11 + 0.193 = 0.303` where 0.11 is the distance from the center of the end effector to the wrist center (frame 5)

Links                      | α<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | θ<sub>i</sub>
---                        | ---             | ---             | ---           | ---
<sup>0</sup>T<sub>1</sub>  | 0               | 0               | 0.75          | q<sub>1</sub>
<sup>1</sup>T<sub>2</sub>  | -pi/2           | 0.35            | 0             | -pi/2 + q<sub>2</sub>
<sup>2</sup>T<sub>3</sub>  | 0               | 1.25            | 0             | q<sub>3</sub>
<sup>3</sup>T<sub>4</sub>  | -pi/2           | -0.054          | 1.50          | q<sub>4</sub>
<sup>4</sup>T<sub>5</sub>  | pi/2            | 0               | 0             | q<sub>5</sub>
<sup>5</sup>T<sub>6</sub>  | -pi/2           | 0               | 0             | q<sub>6</sub>
<sup>6</sup>T<sub>EE</sub> | 0               | 0               | 0.303         | q<sub>EE</sub>

### Deriving Transforms

The DH convention uses 4 indivual transforms:

<sup>i-1</sup>T<sub>i</sub>=R(X<sub>i-1</sub>, α<sub>i-1</sub>)T(X<sub>i-1</sub>, a<sub>i-1</sub>)R(Z<sub>i</sub>, θ<sub>i</sub>)T(Z<sub>i</sub>, d<sub>i</sub>)

Which in matrix form is expressed as:

[dh-transform-matrix]: https://d17h27t6h515a5.cloudfront.net/topher/2017/May/592d6644_dh-transform-matrix/dh-transform-matrix.png
![dh-transform-matrix][dh-transform-matrix]

A slightly less abstract definition of the transform taken from my python code looks like:
```python
Matrix([
    [ cos(q),              -sin(q),             0,           a ],
    [ sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d ],
    [ sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha),  cos(alpha * d) ],
    [ 0,                   0,                   0,           1 ]
])
```

To get the transformation matrix for a given frame we simply replace the values of a, d, q, and alpha with the symbols for that frame and substitute the corresponding DH values, so for example:

For frame `t0_1` we create

```python
Matrix([
    [ cos(q1),          -sin(q1),         0,       0 ],
    [ sin(q1) * cos(0), cos(q1) * cos(0), -sin(0), -sin(0) * 0.75 ],
    [ sin(q1) * sin(0), cos(q1) * sin(0), cos(0),  cos(0 * 0.75) ],
    [ 0,                0,                0,       1 ]
])
```

We can join all of these transforms together to build out a transformation matrix between the base link and end effector by multiplying all the individual transformation matrices.

`t0_7 = t0_1 * t1_2 * t2_3 * t3_4 * t4_5 * t5_6 * t6_7`

We can use this later to validate our thetas obtained via inverse kinematics.


### Inverse Kinematics

The inverse kinematics problem is far more difficult to solve and there are potentially 0 - N many solutions, some of which might not be valid (this depends on the working space/ physical limits of the serial manipulator).

We can kinematically decouple the rotational and positional components of the inverse kinematics problem if the following are satisfied:

1. Three neighboring joint axes intersect at a single point, or
1. Three neighboring joint axes are parallel (which is technically a special case of 1, since parallel lines intersect at infinity)

The serial manipulator we are dealing with, the Kuka 210, satisfies these conditions (it has a spherical wrist) so we can decouple our positional and rotational components.

The spherical wrist (the final 3 joints) will be responsible for rotating the end effector to the correct orientation while the first 3 joints will be responsible for positioning the wrist center (and therefor the end effector) in the right place.

Starting with the end effector (it is our known _goal_ input).

We build out a rotation matrix for the end effector:
```python
# roll, pitch, yaw respectively
r, p, y = symbols('r p y')

# build out the rotation matrices for x, y, and z respectively
rot_x = Matrix([
    [ 1, 0,      0 ],
    [ 0, cos(r), -sin(r) ],
    [ 0, sin(r), cos(r) ]
])

rot_y = Matrix([
    [ cos(p),  0, sin(p) ],
    [ 0,       1, 0 ],
    [ -sin(p), 0, cos(p) ]
])

rot_z = Matrix([
    [ cos(y), -sin(y), 0 ],
    [ sin(y), cos(y),  0 ],
    [ 0,      0,       1 ]
])

(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
    [ req.poses[x].orientation.x, req.poses[x].orientation.y,
        req.poses[x].orientation.z, req.poses[x].orientation.w ])

rot_correction = rot_z.subs(y, radians(180)) * rot_y.subs(p, radians(-90))  # coordinate correction
rot_7 = rot_7 * rot_correction
rot_ee = rot_7.subs({ 'r': roll, 'p': pitch, 'y': yaw })
```

We read the positional values into a matrix for the end effector
```python
end_effector = Matrix([ [px], [py], [pz] ])
```

We then apply a simple translation to get our wrist center
```python
wrist_center = end_effector - (0.303) * rot_ee[:, 2]
```

Now we can begin solving our angles. Starting with `theta1` we can pretty trivially solve it as:

[theta1]: ./theta1.jpg
![theta1][theta1]
```python
theta1 = atan2(wrist_center[1], wrist_center[0])
```

Deriving thetas 2 and 3 is a bit more work:

[theta2_3]: ./theta2_3.jpg
![theta2_3][theta2_3]

First we identify `side_b` as this is a common side for the side-side-side triangles we will use to work out values for thetas 2 and 3. We can then use the cosine laws as well as the values in our DH parameter table to get a value for the theta in question.

`side_b` is the side from joint 2 to joint 4. Joint 4 is the wrist center and we already know that position. The length in Z from joint 2 to 4 is therefore the length from the base to the wrist center, minus the offset from joint 1 to joint 2, in the Z direction. The length in X from 2 to 4 is worked out in a similar way - we know what it is from the base to the wrist center so we can just subtract the offset from joint 1 to joint 2 in X and we have our value. We can then obtain an equation for `side_b` itself.

side_b = sqrt(LX<sub>2-4</sub><sup>2</sup> + LZ<sub>2-4</sub><sup>2</sup>)

We now have two SSS triangles we can use to work out our thetas.

`theta_2` can be worked out by solving the angles `A` and `D` and subtracting those from pi/2.

`theta_3` can be worked out by solving the angle `B` and subtracting that from pi/2 and adjusting for the slight deviation in Z of -0.054 from our DH parameter table.

Recalling that a total homogeneous transform consists of a translation part and a rotation part;

[homogenous-transform]: https://d17h27t6h515a5.cloudfront.net/topher/2017/June/593eef67_eq3/eq3.png
![homogenous-transform][homogenous-transform]

The rotation matrix from the base link to the third link is the rotation part of each homogeneous transform multiplied together and our values for the first 3 angles are substituted in.

```python
rot0_3 = t0_1[0:3, 0:3] * t1_2[0:3, 0:3] * t2_3[0:3, 0:3]
rot0_3 = rot0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
```

Next we can obtain the rotation matrix from link 3 to 6 by taking the transpose of the rotation matrix from link 0 to 2 and multiplying that with the rotation matrix of the end effector.

```python
rot3_6 = rot0_3.transpose() * rot_ee
```

We can then solve the euler angles from the rotation matrix using:

[euler_angles]: https://d17h27t6h515a5.cloudfront.net/topher/2017/August/5981edf6_extrinsicxyz/extrinsicxyz.png
![euler_angles][euler_angles]

Which is done in the python code as:

```python
theta4 = atan2(rot3_6[2, 2], -rot3_6[0, 2])
theta5 = atan2(sqrt(rot3_6[0, 2] * rot3_6[0, 2] + rot3_6[2, 2] * rot3_6[2, 2]), rot3_6[1, 2])
theta6 = atan2(-rot3_6[1, 1], rot3_6[1, 0])
```

We now have all 6 angles for our joints.
