clear all;
clc;
num_drones = 5;
% Dizin yolunu kendi bilgisayarına göre tekrar kontrol et
log_dir = '/home/asd/catkin_ws/src/multi_quadcopter_geometric_control/log/';

% Çıktı dosyası ve klasörü ayarları
output_pdf = 'all_drones_plots_fixed.pdf'; 
eps_folder = 'EPS_Results'; % EPS dosyalarının kaydedileceği klasör

% PDF varsa sil (temiz başlangıç)
if isfile(output_pdf)
    delete(output_pdf);
end

% EPS klasörü yoksa oluştur
if ~exist(eps_folder, 'dir')
    mkdir(eps_folder);
end

colors = lines(num_drones);

% === VERİ SAKLAMA HÜCRELERİ ===
thrust_all = cell(num_drones, 1);
time_all = cell(num_drones, 1);

% İvme verilerini zoom grafiğinde tekrar okumamak için hafızaya alıyoruz
acc_x_all = cell(num_drones, 1);
acc_y_all = cell(num_drones, 1);
acc_z_all = cell(num_drones, 1);
ref_acc_x = cell(num_drones, 1);
ref_acc_y = cell(num_drones, 1);
ref_acc_z = cell(num_drones, 1);


%% === VERİ OKUMA DÖNGÜSÜ ===
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if ~isfile(filename)
        warning('File not found: %s', filename);
        continue;
    end

    data = readtable(filename, 'VariableNamingRule', 'preserve');
    time = data{:, "time"} - data{1, "time"};
    time_all{i} = time;

    % Thrust verilerini sakla
    thrust_all{i} = data{:, "r_thr"};
    
    % İvme verilerini sakla
    acc_x_all{i} = data{:, "a_acc.x"};
    acc_y_all{i} = data{:, "a_acc.y"};
    acc_z_all{i} = data{:, "a_acc.z"};
    
    ref_acc_x{i} = data{:, "r_acc.x"};
    ref_acc_y{i} = data{:, "r_acc.y"};
    ref_acc_z{i} = data{:, "r_acc.z"};
end

% === RENK VE ZAMAN TANIMLARI ===
t_green = [23.5 33.5 33.5 23.5];       color_green = [0.85 1 0.85];
t_yellow = [33.5 60 60 33.5];      color_yellow = [1 1 0.9];

t_zone1 = [0 8 8 0];      c_zone1 = [1 0.88 0.88];    % Pembe
t_zone2 = [8 21 21 8];    c_zone2 = [0.85 1 1];       % Mavi
t_zone3 = [21 33 33 21];  c_zone3 = [0.85 1 0.85];    % Yeşil
t_zone4 = [33 60 60 33];  c_zone4 = [1 1 0.9];        % Sarı


%% === 1. TÜM DRONE'LAR - THRUST ===
fig = figure;
hold on;
lgd_handles = [];
for i = 1:num_drones
    if ~isempty(thrust_all{i})
        h = plot(time_all{i}, thrust_all{i}, 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', sprintf('Quadcopter %d', i));
        lgd_handles = [lgd_handles, h];
    end
end
yl = ylim; yl = [yl(1)*0.9, yl(2)*1.1]; ylim(yl); 
p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
grid off; xlim([0 60]); ylim([3 6.5]);
set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
ylabel('||f_{thrust}|| (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 13, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.15 0.94 0.7 0.05]); 

% KAYDETME
eps_name = fullfile(eps_folder, 'Fig10b.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === 2. TÜM DRONE'LAR - DISTURBANCE FORCES ===
fig = figure;

% --- Fx ---
subplot(3,1,1); hold on;
lgd_handles = [];
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        h = plot(data{:, "time"} - data{1, "time"}, data{:, "dist_f.x"}, 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', sprintf('Quadcopter %d', i));
        lgd_handles = [lgd_handles, h];
    end
end
yl = ylim; yl = [yl(1)*1.1, yl(2)*1.1]; ylim(yl);
p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
ylabel('f_{dist,x} (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% --- Fy ---
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        plot(data{:, "time"} - data{1, "time"}, data{:, "dist_f.y"}, 'LineWidth', 2, 'Color', colors(i,:), 'HandleVisibility', 'off');
    end
end
yl = ylim; yl = [yl(1)*1.1, yl(2)*1.1]; ylim(yl);
p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
ylabel('f_{dist,y} (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% --- Fz ---
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        plot(data{:, "time"} - data{1, "time"}, data{:, "dist_f.z"}, 'LineWidth', 2, 'Color', colors(i,:), 'HandleVisibility', 'off');
    end
end
yl = ylim; yl = [yl(1)*1.1, yl(2)*1.1]; ylim(yl);
p1 = patch(t_green, [yl(1) yl(1) yl(2) yl(2)], color_green, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_yellow, [yl(1) yl(1) yl(2) yl(2)], color_yellow, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1, 'bottom'); uistack(p2, 'bottom');
ylabel('f_{dist,z} (N)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 13, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.1 0.94 0.8 0.05]); 

% KAYDETME
eps_name = fullfile(eps_folder, 'Fig10a.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === 3. TÜM DRONE'LAR - POZİSYON ===
fig = figure;

% === X ===
subplot(3,1,1); hold on;
lgd_handles = [];
h_des = plot(NaN, NaN, ':k', 'LineWidth', 2, 'DisplayName', 'Desired');
lgd_handles = [lgd_handles, h_des];

for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        h = plot(time, data{:, "a_pos.x"}, '-',  'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Quadcopter %d', i));
        lgd_handles = [lgd_handles, h];
        plot(time, data{:, "r_pos.x"}, ':k', 'LineWidth', 2, 'HandleVisibility', 'off');
    end
end
yl = ylim; yl = [yl(1)-1, yl(2)+1]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
ylabel('x (m)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% === Y ===
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        plot(time, data{:, "a_pos.y"}, '-',  'Color', colors(i,:), 'LineWidth', 2);
        plot(time, data{:, "r_pos.y"}, ':k', 'LineWidth', 2);
    end
end
yl = ylim; yl = [yl(1)-1, yl(2)+1]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
ylabel('y (m)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% === Z ===
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        plot(time, data{:, "a_pos.z"}, '-',  'Color', colors(i,:), 'LineWidth', 2);
        plot(time, data{:, "r_pos.z"}, ':k', 'LineWidth', 2);
    end
end
yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
ylabel('z (m)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 13, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.1 0.94 0.8 0.05]); 

% KAYDETME
eps_name = fullfile(eps_folder, 'Fig6a.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === 4. TÜM DRONE'LAR - 3D KONUM ===
fig = figure;
hold on;
lgd_handles = [];
grid off; set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on');
axis equal; axis tight; zlim([0 5]);
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        h = plot3(data{:, "a_pos.x"}, data{:, "a_pos.y"}, data{:, "a_pos.z"}, '-', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Quadcopter %d', i));
        plot3(data{:, "r_pos.x"}, data{:, "r_pos.y"}, data{:, "r_pos.z"}, ':k', 'LineWidth', 2, 'HandleVisibility', 'off');
        lgd_handles = [lgd_handles, h];
    end
end
xlabel('x (m)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
ylabel('y (m)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
zlabel('z (m)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
title('All Drones - 3D Position', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');

view(45, 25);

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 13, 'FontWeight', 'bold', 'Box','off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.1 0.92 0.8 0.05]);

% KAYDETME
eps_name = fullfile(eps_folder, '04_all_drones_3d_position.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === 5. TÜM DRONE'LAR - VELOCITY ===
fig = figure;

% Vx
subplot(3,1,1); hold on;
lgd_handles = [];
h_des = plot(NaN, NaN, ':k', 'LineWidth', 2, 'DisplayName', 'Desired');
lgd_handles = [lgd_handles, h_des];

for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        h = plot(time, data{:, "a_vel.x"}, '-', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Quadcopter %d', i));
        lgd_handles = [lgd_handles, h];
        plot(time, data{:, "r_vel.x"}, ':k', 'LineWidth', 2, 'HandleVisibility', 'off');
    end
end
yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
ylabel('v_x (m/s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% Vy
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        plot(time, data{:, "a_vel.y"}, '-', 'Color', colors(i,:), 'LineWidth', 2);
        plot(time, data{:, "r_vel.y"}, ':k', 'LineWidth', 2);
    end
end
yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
ylabel('v_y (m/s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% Vz
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        plot(time, data{:, "a_vel.z"}, '-', 'Color', colors(i,:), 'LineWidth', 2);
        plot(time, data{:, "r_vel.z"}, ':k', 'LineWidth', 2);
    end
end
yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
ylabel('v_z (m/s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 13, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.1 0.94 0.8 0.05]);

% KAYDETME
eps_name = fullfile(eps_folder, 'Fig7.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === 6. TÜM DRONE'LAR - ACCELERATION (ZOOM) ===
fig = figure;

zoom_t_start = 3.5;
zoom_t_end = 8.5;
zoom_xlim = [zoom_t_start zoom_t_end];

% --- AX ---
h1 = subplot(3,1,1); hold on;
lgd_handles = [];
h_des = plot(NaN, NaN, ':k', 'LineWidth', 2, 'DisplayName', 'Desired');
lgd_handles = [lgd_handles, h_des];

for i = 1:num_drones
    if ~isempty(acc_x_all{i})
        h = plot(time_all{i}, acc_x_all{i}, '-', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Quadcopter %d', i));
        lgd_handles = [lgd_handles, h];
        plot(time_all{i}, ref_acc_x{i}, ':k', 'LineWidth', 2, 'HandleVisibility', 'off');
    end
end
% Arka Plan
yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
% Zoom Dikdörtgeni
rectangle('Position', [zoom_t_start, yl(1)+0.2, zoom_t_end-zoom_t_start, yl(2)-yl(1)-0.4], 'EdgeColor', 'r', 'LineWidth', 2, 'LineStyle', '--');
ylabel('a_x (m/s^2)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); 
set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% >> INSET ZOOM AX
main_pos = get(h1, 'Position'); 
inset_w = main_pos(3) * 0.15;   
inset_h = main_pos(4) * 0.25;   
inset_x = main_pos(1) + main_pos(3)*0.19; 
inset_y = main_pos(2) + main_pos(4)*0.12;
ax_ins = axes('Position', [inset_x inset_y inset_w inset_h]);
box on; hold on;
for i=1:num_drones
    plot(time_all{i}, acc_x_all{i}, 'Color', colors(i,:), 'LineWidth', 2);
    plot(time_all{i}, ref_acc_x{i}, ':k', 'LineWidth', 2);
end
xlim(zoom_xlim); grid off; set(gca, 'Color', 'w', 'FontSize', 8);

% --- AY ---
h2 = subplot(3,1,2); hold on;
for i = 1:num_drones
    if ~isempty(acc_y_all{i})
        plot(time_all{i}, acc_y_all{i}, '-', 'Color', colors(i,:), 'LineWidth', 2);
        plot(time_all{i}, ref_acc_y{i}, ':k', 'LineWidth', 2);
    end
end
% Arka Plan
yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
% Zoom Dikdörtgeni
rectangle('Position', [zoom_t_start, yl(1)+0.2, zoom_t_end-zoom_t_start, yl(2)-yl(1)-0.4], 'EdgeColor', 'r', 'LineWidth', 2, 'LineStyle', '--');
ylabel('a_y (m/s^2)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); 
set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% >> INSET ZOOM AY
main_pos = get(h2, 'Position');
inset_w = main_pos(3) * 0.15;
inset_h = main_pos(4) * 0.25;
inset_x = main_pos(1) + main_pos(3)*0.19;
inset_y = main_pos(2) + main_pos(4)*0.12;
ax_ins = axes('Position', [inset_x inset_y inset_w inset_h]);
box on; hold on;
for i=1:num_drones
    plot(time_all{i}, acc_y_all{i}, 'Color', colors(i,:), 'LineWidth', 2);
    plot(time_all{i}, ref_acc_y{i}, ':k', 'LineWidth', 2);
end
xlim(zoom_xlim); grid off; set(gca, 'Color', 'w', 'FontSize', 8);

% --- AZ ---
h3 = subplot(3,1,3); hold on;
for i = 1:num_drones
    if ~isempty(acc_z_all{i})
        plot(time_all{i}, acc_z_all{i}, '-', 'Color', colors(i,:), 'LineWidth', 2);
        plot(time_all{i}, ref_acc_z{i}, ':k', 'LineWidth', 2);
    end
end
% Arka Plan
yl = ylim; yl = [yl(1)-0.5, yl(2)+0.5]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
% Zoom Dikdörtgeni
rectangle('Position', [zoom_t_start, yl(1)+0.2, zoom_t_end-zoom_t_start, yl(2)-yl(1)-0.4], 'EdgeColor', 'r', 'LineWidth', 2, 'LineStyle', '--');
ylabel('a_z (m/s^2)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); 
set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% >> INSET ZOOM AZ
main_pos = get(h3, 'Position');
inset_w = main_pos(3) * 0.15;
inset_h = main_pos(4) * 0.25;
inset_x = main_pos(1) + main_pos(3)*0.19;
inset_y = main_pos(2) + main_pos(4)*0.12;
ax_ins = axes('Position', [inset_x inset_y inset_w inset_h]);
box on; hold on;
for i=1:num_drones
    plot(time_all{i}, acc_z_all{i}, 'Color', colors(i,:), 'LineWidth', 2);
    plot(time_all{i}, ref_acc_z{i}, ':k', 'LineWidth', 2);
end
xlim(zoom_xlim); grid off; set(gca, 'Color', 'w', 'FontSize', 8);

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 13, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.1 0.94 0.8 0.05]);

% KAYDETME
eps_name = fullfile(eps_folder, 'Fig8.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === 7. TÜM DRONE'LAR - EULER AÇILARI ===
fig = figure;

% --- Roll (Phi) ---
subplot(3,1,1); hold on;
lgd_handles = [];
h_des = plot(NaN, NaN, ':k', 'LineWidth', 2, 'DisplayName', 'Desired');
lgd_handles = [lgd_handles, h_des];
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        
        h = plot(time, data{:, "a_eul_ang.x"}, '-', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Quadcopter %d', i));
        lgd_handles = [lgd_handles, h];
        plot(time, data{:, "r_eul_ang.x"}, ':k', 'LineWidth', 2, 'HandleVisibility', 'off');
    end
end
% Arka Plan
yl = ylim; yl = [yl(1)-0.1, yl(2)+0.1]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
ylabel('\phi (rad)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% --- Pitch (Theta) ---
subplot(3,1,2); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        plot(time, data{:, "a_eul_ang.y"}, '-', 'Color', colors(i,:), 'LineWidth', 2);
        plot(time, data{:, "r_eul_ang.y"}, ':k', 'LineWidth', 2);
    end
end
yl = ylim; yl = [yl(1)-0.1, yl(2)+0.1]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
ylabel('\theta (rad)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

% --- Yaw (Psi) ---
subplot(3,1,3); hold on;
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        plot(time, data{:, "a_eul_ang.z"}, '-', 'Color', colors(i,:), 'LineWidth', 2);
        plot(time, data{:, "r_eul_ang.z"}, ':k', 'LineWidth', 2);
    end
end
yl = ylim; yl = [yl(1)-0.1, yl(2)+0.1]; ylim(yl);
p1 = patch(t_zone1, [yl(1) yl(1) yl(2) yl(2)], c_zone1, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p2 = patch(t_zone2, [yl(1) yl(1) yl(2) yl(2)], c_zone2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p3 = patch(t_zone3, [yl(1) yl(1) yl(2) yl(2)], c_zone3, 'EdgeColor', 'none', 'HandleVisibility', 'off');
p4 = patch(t_zone4, [yl(1) yl(1) yl(2) yl(2)], c_zone4, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(p1,'bottom'); uistack(p2,'bottom'); uistack(p3,'bottom'); uistack(p4,'bottom');
ylabel('\psi (rad)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman'); 
grid off; xlim([0 60]); set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on', 'Layer', 'top');

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 13, 'FontWeight', 'bold', 'Box', 'off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.1 0.94 0.8 0.05]);

% KAYDETME
eps_name = fullfile(eps_folder, 'Fig9.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);


%% === 8. TÜM DRONE'LAR - 3D POZİSYON HATASI ===
fig = figure;
hold on;
lgd_handles = [];
for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        time = data{:, "time"} - data{1, "time"};
        x = data{:, "a_pos.x"};  y = data{:, "a_pos.y"};  z = data{:, "a_pos.z"};
        xr = data{:, "r_pos.x"}; yr = data{:, "r_pos.y"}; zr = data{:, "r_pos.z"};
        pos_error = sqrt((x - xr).^2 + (y - yr).^2 + (z - zr).^2);
        h = plot(time, pos_error, 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', sprintf('Quadcopter %d', i));
        lgd_handles = [lgd_handles, h];
    end
end
xlabel('t (s)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');
ylabel('||e_p|| (m)', 'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman');

grid off; set(gca, 'FontName', 'Times New Roman', 'FontSize', 20, 'FontWeight', 'bold', 'LineWidth', 2, 'Box', 'on');

lgd = legend(lgd_handles, 'Orientation', 'horizontal', 'FontSize', 13, 'FontWeight', 'bold', 'Box','off', 'FontName', 'Times New Roman');
set(lgd, 'Position', [0.1 0.92 0.8 0.05]);

% KAYDETME
eps_name = fullfile(eps_folder, '08_all_drones_position_error.eps');
exportgraphics(fig, eps_name, 'ContentType', 'vector');
exportgraphics(fig, output_pdf, 'Append', true);
close(fig);

% HATA HESAPLAMA EKRAN ÇIKTISI
fprintf('\n=======================================================\n');
fprintf('   PERFORMANS METRİKLERİ (RMSE: Ortalama, ISE: Toplam)   \n');
fprintf('=======================================================\n');

for i = 1:num_drones
    filename = fullfile(log_dir, sprintf('drone%d_log.txt', i));
    if isfile(filename)
        data = readtable(filename, 'VariableNamingRule', 'preserve');
        
        t = data{:, "time"} - data{1, "time"};
        dt = [0; diff(t)]; 
        
        e_x = data{:, "a_pos.x"} - data{:, "r_pos.x"};
        e_y = data{:, "a_pos.y"} - data{:, "r_pos.y"};
        e_z = data{:, "a_pos.z"} - data{:, "r_pos.z"};
        
        sq_error = e_x.^2 + e_y.^2 + e_z.^2;
        rmse_3d = sqrt(mean(sq_error));
        ise_3d = sum(sq_error .* dt); 
        
        fprintf('Quadcopter %d -> RMSE (Ortalama): %.5f m | ISE (Toplam): %.5f\n', i, rmse_3d, ise_3d);
    else
        fprintf('Quadcopter %d: Dosya bulunamadı.\n', i);
    end
end
fprintf('=======================================================\n');
fprintf('Grafikler EPS formatında "EPS_Results" klasörüne kaydedildi.\n');

