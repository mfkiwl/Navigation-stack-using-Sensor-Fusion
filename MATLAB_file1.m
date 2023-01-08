clc;
close all;

%bag file of data in circles
bagCircle = rosbag("data_going_in_cirles.bag");
bagSelCircle = select(bagCircle, "Topic", "/imu");

%bag file of data driving
bagDriv = rosbag("data_driving.bag");
bagSelDriv = select(bagDriv, "Topic", "/imu");

msgStructCircle = readMessages(bagSelCircle, 'DataFormat', 'struct');
magXCircle = cellfun(@(m) double(m.MagField.MagneticField_.X), msgStructCircle);
magYCircle = cellfun(@(m) double(m.MagField.MagneticField_.Y), msgStructCircle);

msgStructDriv = readMessages(bagSelDriv, 'DataFormat', 'struct');
time = cellfun(@(m) double(m.Header.Seq), msgStructDriv);
magXDriv = cellfun(@(m) double(m.MagField.MagneticField_.X), msgStructDriv);
magYDriv = cellfun(@(m) double(m.MagField.MagneticField_.Y), msgStructDriv);
angVelZ = cellfun(@(m) double(m.IMU.AngularVelocity.Z), msgStructDriv);

x = cellfun(@(m) double(m.IMU.Orientation.X), msgStructDriv);
y = cellfun(@(m) double(m.IMU.Orientation.Y), msgStructDriv);
z = cellfun(@(m) double(m.IMU.Orientation.Z), msgStructDriv);
w = cellfun(@(m) double(m.IMU.Orientation.W), msgStructDriv);
wxyz = [w x y z];
time = time - min(time);

bestfitEllipse = fit_ellipse(magXCircle, magYCircle, gca);

translation = [bestfitEllipse.X0_in bestfitEllipse.Y0_in];
rotation = [cos(bestfitEllipse.phi) -sin(bestfitEllipse.phi);sin(bestfitEllipse.phi) cos(bestfitEllipse.phi)];
scaling = [bestfitEllipse.short_axis/bestfitEllipse.long_axis 0;0 1];

coordinatesCircle = [magXCircle magYCircle];
coordinatesDriv = [magXDriv magYDriv];

final_coordinatesCircle = scaling * rotation * transpose(coordinatesCircle  - translation);
final_coordinatesDriv = scaling * rotation * transpose(coordinatesDriv  - translation);

cordXCir = transpose(final_coordinatesCircle(1,:));
cordYCir = transpose(final_coordinatesCircle(2,:));

cordXDir = transpose(final_coordinatesDriv(1,:));
cordYDir = transpose(final_coordinatesDriv(2,:));

figure(1)
plot(magXCircle, magYCircle, 'r');
xlabel("Magnetic X (gauss)");
ylabel("Magnetic Y (gauss)");
title("Magnetometer X-Y before calibration of circle data");
hold on

figure(2)
plot(cordXCir, cordYCir, 'b');
xlabel("Magnetic X (gauss)");
ylabel("Magnetic Y (gauss)");
title("Magnetometer X-Y after calibration of circle data");
hold on

figure(8)
plot(magXDriv, magYDriv, 'r');
xlabel("Magnetic X (gauss)");
ylabel("Magnetic Y (gauss)");
title("Magnetometer X-Y before calibration of driving data");
hold on

figure(9)
plot(cordXDir, cordYDir, 'b');
xlabel("Magnetic X (gauss)");
ylabel("Magnetic Y (gauss)");
title("Magnetometer X-Y after calibration of driving data");
hold on

figure(3)
plot(time, magXDriv, 'r')
hold on
plot(time, cordXDir, 'b')
xlabel("Time (sec)");
ylabel("Magnetic X (gauss)");
legend(["Before calibration", "After calibration"]);
title("Magnetic X vs time");

figure(4)
plot(time, magYDriv, 'r')
hold on
plot(time, cordYDir, 'b')
xlabel("Time (sec)");
ylabel("Magnetic Y (gauss)");
legend(["Before calibration", "After calibration"]);
title("Magnetic Y vs time");

euler = quat2eul(wxyz);
yaw = deg2rad(unwrap(euler(:, 1)));

yaw_gyro = deg2rad(unwrap(cumtrapz(angVelZ)));
% yaw_gyro = yaw_gyro - yaw_gyro(1,1);
yaw_mag = unwrap(atan2(-cordYDir, cordXDir));
% yaw_mag = yaw_mag - yaw_mag(1,1);

figure(5)
plot(time, yaw_gyro, 'r');
hold on
plot(time, yaw_mag, 'b');
xlabel("Time (sec)");
ylabel("Yaw (rads)");
legend(["Yaw from gyroscope", "Yaw from magnetometer"]);
title("Yaw vs Time")

yaw_gyro_hp = highpass(yaw_gyro, 0.00000001);
% yaw_gyro_hp = yaw_gyro_hp - yaw_gyro_hp(1,1);
yaw_mag_lp = lowpass(yaw_mag, 0.01);
% yaw_mag_lp = yaw_mag_lp - yaw_mag_lp(1,1);

alpha = 0.16;
yaw_compl = (1 - alpha) * yaw_mag_lp + alpha * yaw_gyro_hp;

figure(6)
plot(time, yaw_mag_lp, 'y');
hold on
plot(time, yaw_gyro_hp, 'b');
hold on
plot(time, yaw_compl, 'r');
hold on
xlabel("Time (sec)");
ylabel("Yaw (rads)");
legend(["LPF(from mag)", "HPF(from gyro)", "CF"]);
title("LPF, HPF and CF vs time");

figure(7)
plot(time, yaw_compl, 'b');
hold on
plot(time, yaw, 'r');
xlabel("Time (sec)");
ylabel("Yaw (rads)");
legend(["CF", "Yaw from IMU"]);
title("Yaw from CF and IMU vs Time")

