clear all
close all
clc

% Number of expected data per line
packet_size = 14;

% Temporary array
data = [];
index = 1;

% Open log file
fid = fopen('haptic_drone_log_file.txt');

% Untill not end of file
while ~feof(fid)

	% Import next line
	line_ex = fgetl(fid);

	% From string to array
	line_num = str2num(line_ex);

	% Fill data array
	if size(line_num, 2) == packet_size

		if (size(line_num, 1) == 1)
			data(index, :) = line_num;
			index = index + 1;
		end
	end

end

hp = array2table(data,...
    'VariableNames',{'y_c' 'z_c' 'y_d' 'z_d' 'roll_d' 'roll_d_comm' 'effort' 't_x' 'f_y' 'f_z' 'theta' 'f_f' 'f_n' 'mu'});

initial = 12000;
final = size(data,1);

figure;
subplot(4,2,1)
plot(hp.y_c(initial:final), 'LineWidth', 2)
grid on;
hold on
plot(hp.y_d(initial:final), 'LineWidth', 2)
legend('current', 'desired');
title('Lateral Position');
xlabel('samples'); ylabel('y [m]');

subplot(4,2,2)
plot(hp.z_c(initial:final), 'LineWidth', 2)
grid on;
hold on
plot(hp.z_d(initial:final), 'LineWidth', 2)
legend('current', 'desired');
title('Vertical Position');
xlabel('samples'); ylabel('z [m]');

subplot(4,2,3)
plot(hp.roll_d_comm(initial:final), 'LineWidth', 2)
grid on;
title('Commanded Heading');
xlabel('samples'); ylabel('\phi_d [deg]');

subplot(4,2,4)
plot(hp.mu(initial:final), 'LineWidth', 2)
grid on;
title('Friction Coefficient');
xlabel('samples'); ylabel('\mu');

subplot(4,2,5)
plot(hp.f_y(initial:final), 'LineWidth', 2)
grid on;
title('Lateral Force');
xlabel('samples'); ylabel('F_y [N]');

subplot(4,2,7)
plot(hp.f_z(initial:final), 'LineWidth', 2)
grid on;
title('Vertical Force');
xlabel('samples'); ylabel('F_z [N]');

subplot(4,2,6)
plot(hp.f_f(initial:final), 'LineWidth', 2)
grid on;
title('Friction Force');
xlabel('samples'); ylabel('F_F [N]');

subplot(4,2,8)
plot(hp.f_n(initial:final), 'LineWidth', 2)
grid on;
title('Normal Force');
xlabel('samples'); ylabel('F_N [N]');