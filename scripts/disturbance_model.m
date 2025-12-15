clc; clear; close all;

%% 1. Model Parametreleri (Dinamik Ağırlık Tabanlı)

% --- KULLANICI GİRİŞLERİ ---
m_drone = 0.382;    
g = 9.81;           

% --- FİZİKSEL KATSAYILAR ---
C_coup = 0.045;     
r_lat = 0.005;      
rho_air = 1.225;    

% --- HESAPLAMALAR ---
Thrust_per_rotor = m_drone * g / 4; 

K_z = Thrust_per_rotor * C_coup;  
K_x = K_z * r_lat;            
K_y = K_z * r_lat;            

K_matrix = diag([K_x, K_y, K_z]); 

% --- DİĞER PARAMETRELER ---
z0 = 0.1;       
sigma = 0.065;  
turb_intensity = 0.2; 
Pos_Top = [0, 0, 0];  

%% 2. Izgara (Grid) Oluşturma
x_range = -1.5 : 0.02 : 1.5;  
z_range = -2.5 : 0.02 : -0.1;
[X, Z] = meshgrid(x_range, z_range);

%% 3. Hesaplama Döngüsü
F_mag = zeros(size(X));
V_mag = zeros(size(X)); 

rng(42); 

for i = 1:numel(X)
    p_b = [X(i); 0; Z(i)]; 
    p_t = Pos_Top';
    
    p_bt = p_t - p_b; 
    dist = norm(p_bt);
    
    if dist > 0.01
        n_vec = p_bt / dist; 
        d_z = abs(p_bt(3));
        d_xy = norm([p_bt(1); p_bt(2)]);
        
        decay_factor = (1 / (d_z + z0)^2) * exp(-(d_xy^2) / (2*sigma^2));
        f_static = - K_matrix * n_vec * decay_factor;
        
        random_vec = -1 + 2*rand(3, 1);
        f_turbulence = abs(f_static) .* (turb_intensity * random_vec);
        
        f_total = (f_static + f_turbulence);
        F_mag(i) = norm(f_total);
        
        scaling_factor = rho_air * C_coup * 4 + eps; 
        V_mag(i) = sqrt(F_mag(i) / scaling_factor);
    else
        F_mag(i) = 0;
        V_mag(i) = 0;
    end
end

%% 4. Görselleştirme (Times New Roman Font Ayarlı)

figure('Name', 'Downwash Force and Velocity Profiles', 'Color', 'w', 'Position', [100, 100, 1200, 600]);

% --- LAYOUT AYARI ---
t = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact'); 

% --- SOL GRAFİK: KUVVET PROFİLİ ---
nexttile; 
contourf(X, Z, F_mag, 100, 'LineStyle', 'none');
hold on;
plot(0, 0, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'r'); 
% Text nesnesine FontName eklendi
text(0.1, 0.1, 'Source Propeller', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 12, 'FontName', 'Times New Roman');

% Renk Çubuğu Ayarları (FontName eklendi)
cb1 = colorbar; 
set(cb1, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
colormap(jet); caxis([0 max(max(F_mag))]);

% Başlık ve Etiketler (FontName eklendi)
title({'Force Field', ['Max Force: ' num2str(max(max(F_mag)), '%.2f') ' N']}, ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('x (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman'); 
ylabel('z (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

grid off; axis equal; xlim([-0.7 0.7]); ylim([-2.5 0.5]);

% Eksen (Axes) Ayarları (FontName eklendi)
set(gca, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold', ...
    'XColor', 'k', 'YColor', 'k', 'Box', 'on', 'FontName', 'Times New Roman');


% --- SAĞ GRAFİK: HIZ PROFİLİ ---
nexttile; 
contourf(X, Z, V_mag, 100, 'LineStyle', 'none');
hold on;
plot(0, 0, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'r'); 
% Text nesnesine FontName eklendi
text(0.1, 0.1, 'Source Propeller', 'Color', 'k', 'FontWeight', 'bold', 'FontSize', 12, 'FontName', 'Times New Roman');

% Renk Çubuğu Ayarları (FontName eklendi)
cb2 = colorbar; 
set(cb2, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
colormap(jet); caxis([0 max(max(V_mag))]);

% Başlık ve Etiketler (FontName eklendi)
title({'Velocity Field', ['Max Velocity: ' num2str(max(max(V_mag)), '%.2f') ' m/s']}, ...
    'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('x (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman'); 
ylabel('z (m)', 'FontSize', 14, 'FontWeight', 'bold', 'FontName', 'Times New Roman');

grid off; axis equal; xlim([-0.7 0.7]); ylim([-2.5 0.5]);

% Eksen (Axes) Ayarları (FontName eklendi)
set(gca, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold', ...
    'XColor', 'k', 'YColor', 'k', 'Box', 'on', 'FontName', 'Times New Roman');

disp(['Model hesaplandı. Kütle: ' num2str(m_drone) 'kg']);