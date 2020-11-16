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

initial = 2000;
final = 25640;

figure;
subplot(4,1,1)
plot(hp.y_c(initial:final), 'LineWidth', 2)
grid on;
hold on
plot(hp.y_d(initial:final), 'LineWidth', 2)

subplot(4,1,2)
plot(hp.z_c(initial:final), 'LineWidth', 2)
grid on;
hold on
plot(hp.z_d(initial:final), 'LineWidth', 2)

subplot(4,1,3)
%plot(hp.roll_d(initial:final), 'LineWidth', 2)
grid on;
hold on
plot(hp.roll_d_comm(initial:final), 'LineWidth', 2)

subplot(4,1,4)
%plot(hp.theta(initial:final), 'LineWidth', 2)
grid on;
hold on
plot(hp.mu(initial:final), 'LineWidth', 2)

figure;
subplot(4,1,1)
plot(hp.y_c(initial:final), 'LineWidth', 2)
grid on;
subplot(4,1,2)
plot(hp.f_f(initial:final), 'LineWidth', 2)
grid on;
subplot(4,1,3)
plot(hp.f_n(initial:final), 'LineWidth', 2)
grid on;
subplot(4,1,4)
plot(hp.mu(initial:final), 'LineWidth', 2)
grid on;