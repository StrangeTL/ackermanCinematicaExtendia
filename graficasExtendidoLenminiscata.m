%% Lectura de Datos Experimentales
data = dlmread('ackermanExtendido_bernulli_12.txt');
te = data (:,1);
xde = data (:,2);
yde = data (:,3);
xe = data (:,4);
ye = data (:,5);
ve = data (:,6);
we = data (:,7);
phide = data (:,8);
the = data (:,9);
phie = data (:,10);
exe = data (:,11);
eye = data (:,12);
%% Lectura de Datos Simulación
ts = out.timeSim.signals.values(:,:);
xs = out.xpSim.signals.values(:,:);
ys = out.ypSim.signals.values(:,:);
xds = out.xdSim.signals.values(:,:);
yds = out.ydSim.signals.values(:,:);
ths = out.thSim.signals.values(:,:);
phis = out.phiSim.signals.values(:,:);
vs = out.vSim.signals.values(:,:);
ws = out.wSim.signals.values(:,:);
exs = out.exSim.signals.values(:,:);
eys = out.eySim.signals.values(:,:);

%%%%%%%%% Se crea la carpeta para guardar las imagenes
% Nombre de la carpeta y archivo EPS
carpeta = 'graficasLenminiscata';
% Verificar si la carpeta existe, si no, crearla
if ~exist(carpeta, 'dir')
    mkdir(carpeta);
end


%% %%%%%%%%%%%%%%%%%%%%%%%%
figure('units','normalized','outerposition',[0 0 1 1]);
plot(xde,yde,'b','LineWidth', 2);
grid off;
axis equal;
xlabel('X [m]', 'FontSize', 18, 'FontName', 'Arial');
ylabel('Y [m]', 'FontSize', 18, 'FontName', 'Arial');
%title('Trayectorias', 'FontSize', 24, 'FontName', 'Arial');
set(gca, 'FontSize', 18, 'FontName', 'Arial');
xlim([-2.5, 2.5]); % Establece el límite en el eje y desde 0 hasta 30
ylim([-1.5, 1.5]); % Establece el límite en el eje y desde 0 hasta 30


%%
%%%%%%%%% Grafica de las Trayectorias
figure('units','normalized','outerposition',[0 0 1 1]);
plot(xde,yde,'g',xs,ys,'--r',xe,ye,'--b','LineWidth', 2);
hold on;
plot(xe(1), ye(1), 'o', 'MarkerSize', 14, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');  % Pentagrama en el primer punto
plot(xde(1), yde(1), 'o', 'MarkerSize', 14, 'MarkerEdgeColor', 'g', 'MarkerFaceColor', 'g');  % Pentagrama en el primer punto
grid off;
axis equal;
xlabel('X [m]', 'FontSize', 18, 'FontName', 'Arial');
ylabel('Y [m]', 'FontSize', 18, 'FontName', 'Arial');
%title('Trayectorias', 'FontSize', 24, 'FontName', 'Arial');
legend('Deseada', 'Simulación','Expermiental','Position',[0.330405083413587 0.932089166668427 0.372620782862645 0.0497667172092301],...
    'Orientation','horizontal','FontSize', 18, 'FontName', 'Arial');
set(gca, 'FontSize', 18, 'FontName', 'Arial');
xlim([-2.5, 2.5]); % Establece el límite en el eje y desde 0 hasta 30
ylim([-1.5, 1.5]); % Establece el límite en el eje y desde 0 hasta 30

% Nombre del archivo EPS
nombre_archivo = 'lenminiscata_grafica_trayectoria.eps';
% Guardar la gráfica en formato EPS dentro de la carpeta
nombre_completo = fullfile(carpeta, nombre_archivo);
saveas(gcf, nombre_completo, 'epsc');
% Nombre del archivo JPG
nombre_archivo_jpg = 'lenminiscata_grafica_trayectoria.jpg';
% Guardar la gráfica en formato JPG dentro de la carpeta
nombre_completo_jpg = fullfile(carpeta, nombre_archivo_jpg);
saveas(gcf, nombre_completo_jpg, 'jpg');

%%
%%%%%%%%% Grafica del Error en X
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2, 1, 1); % Subplot para la primera gráfica
plot(ts,exs,'r',te,exe,'b','LineWidth', 2);
yticks([ -0.15, -0.075, 0, 0.075, 0.15]);
grid off;
%set(gca, 'YGrid', 'on'); 
xlabel('t [s]', 'FontSize', 20, 'FontName', 'Arial');
ylabel('ex [m]', 'FontSize', 20, 'FontName', 'Arial');
title('Error en X', 'FontSize', 20, 'FontName', 'Arial');
legend('simulación','Expermiental','FontSize', 20, 'FontName', 'Arial','Position',[0.622469142027843 0.918024691358025 0.284444438219071 0.0518518504831526],...
    'Orientation','horizontal');
set(gca, 'FontSize', 20, 'FontName', 'Arial');
%xlim([0, 30]); % Establece el límite en el eje y desde 0 hasta 30
ylim([-0.15, 0.15]); % Establece el límite en el eje y desde 0 hasta 30
%%%%%%%%% Grafica del Error en Y
%figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2, 1, 2); % Subplot para la segunda gráfica
plot(ts,eys,'r',te,eye,'b','LineWidth', 2);
yticks([ -0.30, -0.15, 0, 0.15, 0.30]);
grid off;
%set(gca, 'YGrid', 'on'); 
xlabel('t [s]', 'FontSize', 20, 'FontName', 'Arial');
ylabel('ey [m]', 'FontSize', 20, 'FontName', 'Arial');
title('Error en Y', 'FontSize', 20, 'FontName', 'Arial');
legend('Simulación','Expermiental','FontSize', 20, 'FontName', 'Arial','Position',[0.621728401287102 0.442469135802469 0.284444438219071 0.0518518504831527],...
    'Orientation','horizontal');
set(gca, 'FontSize', 20, 'FontName', 'Arial');
%xlim([0, 30]);    % Establece el límite en el eje x
ylim([-0.3, 0.3]); % Establece el límite en el eje y

% Nombre del archivo EPS
nombre_archivo = 'lenminiscata_grafica_error.eps';
% Guardar la gráfica en formato EPS dentro de la carpeta
nombre_completo = fullfile(carpeta, nombre_archivo);
saveas(gcf, nombre_completo, 'epsc');
% Nombre del archivo JPG
nombre_archivo_jpg = 'lenminiscata_grafica_error.jpg';
% Guardar la gráfica en formato JPG dentro de la carpeta
nombre_completo_jpg = fullfile(carpeta, nombre_archivo_jpg);
saveas(gcf, nombre_completo_jpg, 'jpg');

%%
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2, 1, 1); % Subplot para la segunda gráfica
plot(ts,vs,'r',te,ve,'b','LineWidth', 2);
yticks([0,0.1, 0.2,0.3]);
grid off;
%set(gca, 'YGrid', 'on'); 
xlabel('t [s]', 'FontSize', 20, 'FontName', 'Arial');
ylabel('v [m/s]', 'FontSize', 20, 'FontName', 'Arial');
title('Control v', 'FontSize', 20, 'FontName', 'Arial');
legend('Simulación','Expermiental','FontSize', 20, 'FontName', 'Arial','Position',[0.621728401287103 0.916543209876543 0.284444438219071 0.0518518504831526],...
    'Orientation','horizontal');
set(gca, 'FontSize', 20, 'FontName', 'Arial');
%xlim([0, 30]); % Establece el límite en el eje x
ylim([0.0, 0.3]); % Establece el límite en el eje y

%%
subplot(2, 1, 2); % Subplot para la segunda gráfica
plot(ts,ws,'r',te,we,'b','LineWidth', 2);
yticks([ -2.0, -1.0, 0, 1.0, 2.0]);
grid off;
%set(gca, 'YGrid', 'on'); 
xlabel('t [s]', 'FontSize', 20, 'FontName', 'Arial');
ylabel('w [rad/s]', 'FontSize', 20, 'FontName', 'Arial');
title('Control w', 'FontSize', 20, 'FontName', 'Arial');
legend('Simulación','Expermiental','FontSize', 20, 'FontName', 'Arial','Position',[0.620987660546361 0.44395061728395 0.284444438219071 0.0518518504831527],...
    'Orientation','horizontal');
set(gca, 'FontSize', 20, 'FontName', 'Arial');
%xlim([0, 30]); % Establece el límite en el eje x
ylim([-2.0, 2.0]); % Establece el límite en el eje y

% Nombre del archivo EPS
nombre_archivo = 'lenminiscata_grafica_control.eps';
% Guardar la gráfica en formato EPS dentro de la carpeta
nombre_completo = fullfile(carpeta, nombre_archivo);
saveas(gcf, nombre_completo, 'epsc');
% Nombre del archivo JPG
nombre_archivo_jpg = 'lenminiscata_grafica_control.jpg';
% Guardar la gráfica en formato JPG dentro de la carpeta
nombre_completo_jpg = fullfile(carpeta, nombre_archivo_jpg);
saveas(gcf, nombre_completo_jpg, 'jpg');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
figure('units','normalized','outerposition',[0 0 1 1]);
plot(ts,vs,'r',te,ve,'b','LineWidth', 2);
yticks([0,0.05,0.1,0.15,0.2,0.25]);
grid off;
%set(gca, 'YGrid', 'on'); 
xlabel('t [s]', 'FontSize', 20, 'FontName', 'Arial');
ylabel('v [m/s]', 'FontSize', 20, 'FontName', 'Arial');
title('Control v', 'FontSize', 20, 'FontName', 'Arial');
legend('Simulación','Expermiental','FontSize', 20, 'FontName', 'Arial','Position',[0.616913586527478 0.928395061728395 0.288148141812395 0.0518518504831524],...
    'Orientation','horizontal');
set(gca, 'FontSize', 20, 'FontName', 'Arial');
xlim([0, 200]); % Establece el límite en el eje x
ylim([0.0, 0.25]); % Establece el límite en el eje y

% Nombre del archivo EPS
nombre_archivo = 'lenminiscata_grafica_control_v.eps';
% Guardar la gráfica en formato EPS dentro de la carpeta
nombre_completo = fullfile(carpeta, nombre_archivo);
saveas(gcf, nombre_completo, 'epsc');
% Nombre del archivo JPG
nombre_archivo_jpg = 'lenminiscata_grafica_control_v.jpg';
% Guardar la gráfica en formato JPG dentro de la carpeta
nombre_completo_jpg = fullfile(carpeta, nombre_archivo_jpg);
saveas(gcf, nombre_completo_jpg, 'jpg');


%%
figure('units','normalized','outerposition',[0 0 1 1]);
plot(ts,ws,'r',te,we ,'b','LineWidth', 2);
yticks([-3.0, 0.0 , 6.0, 12.0,  18.0, 24.0]);
grid off;
%set(gca, 'YGrid', 'on'); 
xlabel('t [s]', 'FontSize', 20, 'FontName', 'Arial');
ylabel('w [rad/s]', 'FontSize', 20, 'FontName', 'Arial');
title('Control w', 'FontSize', 20, 'FontName', 'Arial');
legend('Simulación','Expermiental','FontSize', 20, 'FontName', 'Arial','Position',[0.617654327268218 0.929876543209877 0.288148141812395 0.0518518504831526],...
    'Orientation','horizontal');
set(gca, 'FontSize', 20, 'FontName', 'Arial');
xlim([0, 200]); % Establece el límite en el eje x
ylim([-3.0, 24.0]); % Establece el límite en el eje y

% Nombre del archivo EPS
nombre_archivo = 'lenminiscata_grafica_control_w.eps';
% Guardar la gráfica en formato EPS dentro de la carpeta
nombre_completo = fullfile(carpeta, nombre_archivo);
saveas(gcf, nombre_completo, 'epsc');
% Nombre del archivo JPG
nombre_archivo_jpg = 'lenminiscata_grafica_control_w.jpg';
% Guardar la gráfica en formato JPG dentro de la carpeta
nombre_completo_jpg = fullfile(carpeta, nombre_archivo_jpg);
saveas(gcf, nombre_completo_jpg, 'jpg');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure('units','normalized','outerposition',[0 0 1 1]);
% plot(ts,phis,'r',te,phide,'b','LineWidth', 2);
% yticks([ -1.0, -0.5, 0.0 , 0.5, 1.0]);
% grid off;
% %set(gca, 'YGrid', 'on'); 
% xlabel('t [s]', 'FontSize', 20, 'FontName', 'Arial');
% ylabel('\phi [rad]', 'FontSize', 20, 'FontName', 'Arial');
% title('Ángulo de dirección \phi', 'FontSize', 20, 'FontName', 'Arial');
% legend('Simulación','Expermiental','FontSize', 20, 'FontName', 'Arial','Position',[0.617654327268218 0.929876543209877 0.288148141812395 0.0518518504831526],...
%     'Orientation','horizontal');
% set(gca, 'FontSize', 20, 'FontName', 'Arial');
% xlim([0, 200]); % Establece el límite en el eje x
% ylim([-1.0, 1.0]); % Establece el límite en el eje y

% % Nombre del archivo EPS
% nombre_archivo = 'lenminiscata_grafica_control_phi.eps';
% % Guardar la gráfica en formato EPS dentro de la carpeta
% nombre_completo = fullfile(carpeta, nombre_archivo);
% saveas(gcf, nombre_completo, 'epsc');
% % Nombre del archivo JPG
% nombre_archivo_jpg = 'lenminiscata_grafica_control_phi.jpg';
% % Guardar la gráfica en formato JPG dentro de la carpeta
% nombre_completo_jpg = fullfile(carpeta, nombre_archivo_jpg);
% saveas(gcf, nombre_completo_jpg, 'jpg');
