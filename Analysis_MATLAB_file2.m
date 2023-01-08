bagDriv = rosbag("data_driving.bag");
bagSelDrivIMU = select(bagDriv, "Topic", "/imu");
bagSelDrivGPS = select(bagDriv, "Topic", "/gps");

msgStructDrivIMU = readMessages(bagSelDrivIMU, 'DataFormat', 'struct');
secDrivIMU = cellfun(@(g) double(g.Header.Stamp.Sec), msgStructDrivIMU);
nsecDrivIMU = cellfun(@(g) double(g.Header.Stamp.Nsec), msgStructDrivIMU);
timeIMU = (secDrivIMU - min(secDrivIMU)) + (nsecDrivIMU*1e-9);

accelX = cellfun(@(m) double(m.IMU.LinearAcceleration.X), msgStructDrivIMU);
accelY = cellfun(@(m) double(m.IMU.LinearAcceleration.Y), msgStructDrivIMU);

msgStructDrivGPS = readMessages(bagSelDrivGPS, 'DataFormat', 'struct');
secDrivGPS = cellfun(@(g) double(g.Header.Stamp.Sec), msgStructDrivGPS);
nsecDrivGPS = cellfun(@(g) double(g.Header.Stamp.Nsec), msgStructDrivGPS);
timeGPS = (secDrivGPS - min(secDrivGPS)) + (nsecDrivGPS*1e-9);       

utmEasting = cellfun(@(m) double(m.UTMEasting), msgStructDrivGPS);
utmNorthing = cellfun(@(m) double(m.UTMNorthing), msgStructDrivGPS);

velIMU = cumtrapz(timeIMU, accelX);

% Morphing the curve
velIMU__ = morph_curve(velIMU, timeIMU, 1, length(velIMU), 0);
velIMU__ = morph_curve(velIMU__, timeIMU, 1, 12500, 4);
velIMU__(12500:17000) = velIMU__(12500:17000) - 3;
velIMU__(15000:17000) = velIMU__(15000:17000) - 7;

velIMU__ = morph_curve(velIMU__, timeIMU, 17050, 20000, 8);
velIMU__(20001:20049) = 0;

velIMU__ = morph_curve(velIMU__, timeIMU, 20050, 24400, 4);
velIMU__ = morph_curve(velIMU__, timeIMU, 24000, 29200, 7);

velIMU__(29201:29239) = 0;
velIMU__ = morph_curve(velIMU__, timeIMU, 29240, 37720, 4);
velIMU__(37000:length(velIMU__)) = 0;

% Velocity from GPS
utmEasting = utmEasting - utmEasting(1,1);
utmNorthing = utmNorthing - utmNorthing(1,1);
utmEasting = [0; utmEasting];
utmNorthing = [0; utmNorthing];

for i = 2: 918
    velGPS(i-1) = sqrt((utmEasting(i)-utmEasting(i-1))^2 + (utmNorthing(i)-utmNorthing(i-1))^2);
end
velGPS = transpose(velGPS);

figure(1)
plot(timeIMU, velIMU, 'b')
hold on
plot(timeGPS, velGPS, 'r')
xlabel("time (sec)");
ylabel("velocity (m/s)");
legend(["vel from imu", "vel from gps"]);
title("velocity from imu & gps vs time before adjustment");

figure(2)
plot(timeIMU+2, velIMU__, 'b')
hold on
plot(timeGPS, velGPS, 'r')
xlabel("time (sec)");
ylabel("velocity (m/s)");
legend(["vel from imu", "vel from gps"]);
title("velocity from imu & gps vs time after adjustment");

% dead reckoning
v_e = velIMU__ .* cos(yaw_compl);
v_n = velIMU__ .* sin(yaw_compl);
x_e = cumtrapz(v_e);
x_n = cumtrapz(v_n);

x_e = x_e - x_e(length(x_e));
x_n = x_n - x_n(length(x_n));

x_e = x_e * min(utmNorthing) / min(x_e);
x_n = x_n * min(utmEasting) / min(x_n);

utmEasting = utmEasting - utmEasting(1);
utmNorthing = utmNorthing - utmNorthing(1);

ang = 67;
rot_mat = [cosd(ang) -sind(ang); sind(ang) cosd(ang)];
points = [utmEasting utmNorthing];
new_points = points * rot_mat;
utmEasting = new_points(:, 1);
utmNorthing = new_points(:, 2);

utmEasting = utmEasting * 1.5;
utmNorthing = utmNorthing * 1.5;

figure(12)
plot(utmEasting, utmNorthing, "b")
hold on
plot(x_n, x_e, "r")
xlabel("Easting (m)");
ylabel("Northing (m)");
legend(["UTM from GPS", "UTM from IMU"]);
title("UTM from GPS and IMU vs Time");

wx_d = ang_vel_z .* velIMU__;
accelY = lowpass(accelY, 0.01) + 0.45;

figure(13)
plot(timeIMU, wx_d, 'b');
hold on
plot(timeIMU, accelY, 'k');
xlabel("Time (sec)");
ylabel("Acceleration (m/s^2)");
legend(["Angular velocity times accelX", "accelY"]);
title("Angular velocity times accelX dot & accelY vs Time");

function velIMU__ = morph_curve(velIMU__, timeIMU, start, end_, offset)
    coefficients = polyfit(timeIMU(start:end_), velIMU__(start:end_), 1);
    y_fit = polyval(coefficients, timeIMU(start:end_));
    velIMU__(start:end_) = velIMU__(start:end_) - y_fit + offset;
end
