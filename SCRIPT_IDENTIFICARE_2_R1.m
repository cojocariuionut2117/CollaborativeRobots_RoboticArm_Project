%% Citește datele din fișier
data = load('date_IDENTIFICARE_LICENTA.txt');

% Separă coloanele
col1 = data(:, 1);
col2 = data(:, 2);
col3 = data(:, 3);

% Afișează fiecare coloană
% disp('Coloana 1:');
% disp(col1);
% 
% disp('Coloana 2:');
% disp(col2);
% 
% disp('Coloana 3:');
% disp(col3);

impulsuri1 = col1';
impulsuri2 = col2';

windowSize = 3;
impulsuri1 = movmean(impulsuri1, windowSize);
impulsuri2 = movmean(impulsuri2, windowSize);
comanda = col3';
t = 0:0.1:3.1;

%% masurare viteza
hold on
figure(10)
subplot 211
plot(t, impulsuri2, LineWidth=2)
grid on
xlim([0 3])
ylim([0 120])
xlabel('timp[secunde]')
ylabel('viteza [impulsuri / secundă]')

subplot 212
plot(t,comanda*100/255, LineWidth=2);
grid on
xlabel('timp[secunde]')
ylabel('comanda PWM [%]')
xlim([0 3])
ylim([0 100])
%%

%plot(t, impulsuri1); folosim doar setul 2
hold on
plot(t, impulsuri2, LineWidth=1)
plot(t,comanda, LineWidth=1);
grid on


K = (100 - 60) / (220 - 130);
y63 = 0.63*(100 - 60) + 60;
T = 0.4;

Hf = tf(K, [T 1]);
[y_sim,t] = step(Hf,t);

plot(t+0.8, y_sim*90+60, LineWidth=1)
xline(0.8)
legend('Nr impulsuri/sec','Comanda', 'Y simulat', 'Momentul treptei')

%% calcul regulator
%Guillemin - Truxal
suprareglaj = 2;
ts = 0.6;

tita = log(suprareglaj)/(sqrt(log(suprareglaj)^2 + pi^2));
wn = 4/tita/ts;
s = tf('s');
Ho = wn^2/(s^2 + 2*tita*wn*s + wn^2);
Hr = 1/Hf * Ho/(1-Ho);
Hr = zpk(Hr);

Hr_simplificat = tf([861.69 861.69*2.5],[1 13.33 0]);
[Kp, Ki, Kd] = piddata(Hr_simplificat)