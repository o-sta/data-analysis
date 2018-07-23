% データ読み込みとpdf書き出し
clear
close all


% 設定項目
min_time = 2;
max_time = 22;
% fname = 'h5';
% fpathname = 'hover/';
fname = 'm_z';
fpathname = 'move2/';
filepath_bag = strcat('./',fpathname,fname,'.bag');


bag = rosbag(filepath_bag);
bsel_nav = select(bag,'Topic','/ardrone/navdata');
bsel_tr = select(bag,'Topic','/transition');

motor = timeseries(bsel_nav,'Motor1','Motor2','Motor3','Motor4');   %モータのPWMパラメータ
v = timeseries(bsel_nav,'Vx','Vy','Vz');                            %速度
a = timeseries(bsel_nav,'Ax','Ay','Az');                            %加速度

filapath_csv = strcat(fpathname, fname, '.csv');
cmd_data = csvread(filapath_csv,1,0);     %コマンド読み込み
cmd_data(:,1) = cmd_data(:,1)*10^-9;

% 時間を推移させる
cmd_data = relative_time_array(cmd_data,a.Time(1));
a = relative_time(a);
v = relative_time(v);
motor = relative_time(motor);

%開始時間を特定
i=1;
while a.Time(i) < min_time
    i = i + 1;
end
op = a.Time(i);
%データのカット
cmd_data = cut_data_timebase_array(cmd_data, min_time, max_time, op);
a = cut_data_timebase(a,min_time,max_time);
v = cut_data_timebase(v,min_time,max_time);
motor = cut_data_timebase(motor,min_time,max_time);

%コマンドを切り替えた時間のデータを作成
s = numel(cmd_data(:,2));
k = -2;
n = 1;
for i = 1:s
    if cmd_data(i,2) ~= k
        cmd_pick(n) = cmd_data(i,1);
        k = cmd_data(i,2);
        n=n+1;
    end
end
cmd_pick(n) = cmd_data(i,1);

% プロット
fig1 = figure(1);
movegui('northwest')
hold on
p1 = plot(motor,'LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('時間 [s]','FontName','arial','FontSize',16)
ylabel('PWM値','FontName','arial','FontSize',16)
ylim manual;
for i = 1:numel(cmd_pick(:))
    k = cmd_pick(i);
    plot([k,k],[-5000,5000],'color',[0.5 0.5 0.5])
end
legend([p1],'m1','m2','m3','m4','location','south')
set(legend,'FontName','arial','FontSize',14)


fig2 = figure(2);
iptwindowalign(fig1,'right',fig2,'left');
hold on
p2 = plot(v,'LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('時間 [s]','FontName','arial','FontSize',16)
ylabel('速度[mm/sec]','FontName','arial','FontSize',16)
ylim manual;
for i = 1:numel(cmd_pick(:))
    k = cmd_pick(i);
    plot([k,k],[-5000,5000],'color',[0.5 0.5 0.5])
end
legend([p2],'vx','vy','vz','location','northwest')
set(legend,'FontName','arial','FontSize',16)

fig3 = figure(3);
iptwindowalign(fig2,'right',fig3,'left');
hold on
p3 = plot(a,'LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('時間 [s]','FontName','arial','FontSize',16)
ylabel('加速度[m/s^2*0.1]','FontName','arial','FontSize',16)
ylim manual;
for i = 1:numel(cmd_pick(:))
    k = cmd_pick(i);
    plot([k,k],[-5000,5000],'color',[0.5 0.5 0.5])
end
legend([p3],'ax','ay','az','location','northwest')
set(legend,'FontName','arial','FontSize',16)


%保存
% 
% saveas(fig1,'./2018_6_28/pdf/hover_motor.pdf')
% saveas(fig2,'./2018_6_28/pdf/hover_v.pdf')
% saveas(fig3,'./2018_6_28/pdf/hover_a.pdf')
% 
% saveas(fig1,'./2018_6_28/pdf/gb_motor.pdf')
% saveas(fig2,'./2018_6_28/pdf/gb_v.pdf')
% saveas(fig3,'./2018_6_28/pdf/gb_a.pdf')」

% 移動平均プログラム
MEAN_VALUE = 3; %移動平均の値

motor_m = motor;
v_m = v;
a_m = a;

motor_m.Data = movmean(motor.Data,MEAN_VALUE,1);
a_m.Data = movmean(a.Data,MEAN_VALUE,1);
v_m.Data = movmean(v.Data,MEAN_VALUE,1);

% プロット 移動平均
fig4 = figure(4);
iptwindowalign(fig1, 'bottom', fig4, 'top');
iptwindowalign(fig1,'hcenter',fig4,'hcenter');
hold on
p4 = plot(motor_m,'LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('時間 [s]','FontName','arial','FontSize',16)
ylabel('PWM値','FontName','arial','FontSize',16)
ylim manual;
for i = 1:numel(cmd_pick(:))
    k = cmd_pick(i);
    plot([k,k],[-5000,5000],'color',[0.5 0.5 0.5])
end
legend([p4],'m1','m2','m3','m4','location','south')
set(legend,'FontName','arial','FontSize',14)

fig5 = figure(5);
iptwindowalign(fig2, 'bottom', fig5, 'top');
iptwindowalign(fig2,'hcenter',fig5,'hcenter');
hold on
p5 = plot(v_m,'LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('時間 [s]','FontName','arial','FontSize',16)
ylabel('速度[mm/sec]','FontName','arial','FontSize',16)
ylim manual;
for i = 1:numel(cmd_pick(:))
    k = cmd_pick(i);
    plot([k,k],[-5000,5000],'color',[0.5 0.5 0.5])
end
legend([p5],'vx','vy','vz','location','northwest')
set(legend,'FontName','arial','FontSize',16)

fig6 = figure(6);
iptwindowalign(fig3, 'bottom', fig6, 'top');
iptwindowalign(fig3,'hcenter',fig6,'hcenter');
hold on
p6 = plot(a_m,'LineWidth',2);
set(gca,'FontName','arial','FontSize',14)
xlabel('時間 [s]','FontName','arial','FontSize',16)
ylabel('加速度[m/s^2*0.1]','FontName','arial','FontSize',16)
ylim manual;
for i = 1:numel(cmd_pick(:))
    k = cmd_pick(i);
    plot([k,k],[-5000,5000],'color',[0.5 0.5 0.5])
end
legend([p6],'ax','ay','az','location','northwest')
set(legend,'FontName','arial','FontSize',16)


% ファイルの保存
savef1 = strcat('./figure/', fname, '_motor.pdf');
savef2 = strcat('./figure/', fname, '_v.pdf');
savef3 = strcat('./figure/', fname, '_a.pdf');
savef4 = strcat('./figure/', fname, '_motor_m.pdf');
savef5 = strcat('./figure/', fname, '_v_m.pdf');
savef6 = strcat('./figure/', fname, '_a_m.pdf');

saveas(fig1,savef1)
saveas(fig2,savef2)
saveas(fig3,savef3)
saveas(fig4,savef4)
saveas(fig5,savef5)
saveas(fig6,savef6)

