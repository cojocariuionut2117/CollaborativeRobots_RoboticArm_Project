%% Citește datele din fișier
data = load('date_IDENTIFICARE_LICENTA_R2.txt');

% Separă coloanele
col1 = data(:, 1);
col2 = data(:, 2);

impulsuri1 = col1';
impulsuri2 = col2';
comanda(1:18) = 130;
comanda(19:length(impulsuri1)) = 220;

windowSize = 3;
impulsuri1 = movmean(impulsuri1, windowSize);
impulsuri2 = movmean(impulsuri2, windowSize);
t = 0:0.1:3.7;

%% identificare
figure(10)
% title('Raspunsul la treapta pentru viteza lui CarryBot')

subplot 211
plot(t, comanda*100/255, LineWidth=2); 
grid on
xlim([0 3.5])
ylim([0 100])
xlabel('timp[secunde]')
ylabel('comanda PWM [%]')

subplot 212
plot(t,impulsuri1, LineWidth=2);
grid on
xlim([0 3.5])
ylim([0 130])
xlabel('timp[secunde]')
ylabel('viteza [impulsuri / secundă]')
%%

plot(t, impulsuri1, LineWidth=2); 
hold on
% plot(t, impulsuri2, LineWidth=2) %folosim doar setul 1
% plot(t,comanda, LineWidth=2);
xline(1.7, LineWidth=1)
% yline(0.63*(110 - 70) + 70)
grid on

K = (110 - 70) / (220 - 130);
y63 = 0.63*(110 - 70) + 70;
T = 0.17; % timpul pana la 63%

Hf = tf(K, [T 1]);
[y_sim,t] = step(Hf,t);

plot(t+1.7, y_sim*90+70, LineWidth=2)
xlim([0 3.7])

legend('Viteza [impulsuri / secunda]','Momentul treptei','Viteza simulata [impulsuri / secunda]')

%% calcul regulator
%Guillemin - Truxal
suprareglaj = 0.2;
ts = 0.6;

tita = log(suprareglaj)/(sqrt(log(suprareglaj)^2 + pi^2));
wn = 4/tita/ts;
s = tf('s');
Ho = wn^2/(s^2 + 2*tita*wn*s + wn^2);
Hr = 1/Hf * Ho/(1-Ho);
Hr = zpk(Hr)

Hr_simplificat = tf([81.774 81.774*5.882],[1 13.33 0]);
[Kp, Ki, Kd] = piddata(Hr_simplificat)