warning('off','all');
clear
close all
clc
%--------------------------------------------------------------------------
%% Vari?veis
global xobs_2 Ko_2 C_a_2 phi_a_2 gama_a_2; % Realimentar 
global xobs Ko C_a phi_a gama_a; % Observador de estado aumentado - Observar
Ko=[0 0 0]';
C_a = [0 0 0];
phi_a = zeros(3,3);
gama_a = zeros(3,1);
xobs=[0;0;0]; 
Ko_2=[0 0 0]';
C_a_2 = [0 0 0];
phi_a_2 = zeros(3,3);
gama_a_2 = zeros(3,1);
xobs_2=[0;0;0]; 
La = zeros(1,3);
Lc = 0;
perturbacao_fv = 1;
perturbacao_st = 8;
perturbacao_bin_fv = 0;
perturbacao_bin_st = 0;
%--------------------------------------------------------------------------
%% Alineas 1,2,3,4,5
Rm=2.6;%ohms
Km=0.0078;%Nm/A
Lm=0.00018;%H
Kgi=14; %relacao entre engrenagens e caixa redutora 
Kge=5; %relacao entre engrenagens do trem externo
Jmrotor=3.9e-7; %kgm^2
Jm=4.60625e-7; %Kg.m^2
JL=6.63e-5; %Kg.m^2
Jg=5.28e-5; %Kg.m2;
Kg=Kge*Kgi;
Jeq= Kg^2*Jm +JL;
Ksi=0.7;
tp=0.2;
wd=pi/tp;
wn=wd/sqrt(1-Ksi^2);
h = 0.01;
num = 1;
den = [(Rm*Jeq)/(Km*Kg) Km*Kg];
a0 = 0;
b0 = (Km*Kg)/(Rm*Jeq);
a1 = ((Km*Kg)^2)/(Rm*Jeq);

P = 70;

Kp = ((2*P*Ksi*wn)+wn^2-a0)/(b0);
Kd = ((2*Ksi*wn)+P-a1)/(b0);
Ki = 0;

Jl=JL;
Kf=0;
Ke=Km;
Bl=0;
% Modelo do motor State Space
A = [0 1; 0 -0.5460/0.0111];
B = [0 1/0.0111]';
C = [1 0];
D = 0;

Ca = [1 0; 0 1];
Da = [0 0]';
C1 = [1 0];
C2 = [0 1];
% Fim do motor State Space
%--------------------------------------------------------------------------
%% Alinea 4
% 1)
filtro = 0;
sim('servermotor');
% Motor DC + Engrenagens + Carga
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_1.time, teta_1.signals.values,'r',teta_ponto_1.time, teta_ponto_1.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PD - Motor DC + Engrenagens + Carga - Sem Filtro')
% Funcao Transferencia Motor DC
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_2.time, teta_2.signals.values,'r',teta_ponto_2.time, teta_ponto_2.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PD - Funcao Transferencia Motor DC - Sem Filtro ')
% Espaco de Estados
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',x1.time, x1.signals.values,'r',x2.time, x2.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PD - Modelo em Espaco de Estados - Sem Filtro');
% Comparacao entre as Posicoes Angulares dos tres modelos
figure
plot(teta_1.time, teta_1.signals.values,'r',teta_2.time, teta_2.signals.values,'b',x1.time, x1.signals.values,'k');
legend('Motor DC + Engrenagens + Carga','Funcao Transferencia Motor DC','Modelo em Espaco de Estados');
title('Controlo do tipo PD - Comparacao da Posicao Angular dos tres modelos - Sem Filtro');
% Comparacao entre as Velocidades Angulares dos tres modelos
figure
plot(teta_ponto_1.time, teta_ponto_1.signals.values,'r',teta_ponto_2.time, teta_ponto_2.signals.values,'b',x2.time, x2.signals.values,'k');
legend('Motor DC + Engrenagens + Carga','Funcao Transferencia Motor DC','Modelo em Espaco de Estados');
title('Controlo do tipo PD - Comparacao as Velocidades Angulares dos tres modelos - Sem Filtro');
% 2)
filtro = 100;
sim('servermotor');
% Motor DC + Engrenagens + Carga
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_1.time, teta_1.signals.values,'r',teta_ponto_1.time, teta_ponto_1.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PD - Motor DC + Engrenagens + Carga - Com Filtro')
% Funcao Transferencia Motor DC
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_2.time, teta_2.signals.values,'r',teta_ponto_2.time, teta_ponto_2.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PD - Funcao Transferencia Motor DC - Com Filtro ')
% Espaco de Estados
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',x1.time, x1.signals.values,'r',x2.time, x2.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PD - Modelo em Espaco de Estados - Com Filtro');
% Comparacao entre as Posic??es Angulares dos tres modelos
figure
plot(teta_1.time, teta_1.signals.values,'r',teta_2.time, teta_2.signals.values,'b',x1.time, x1.signals.values,'k');
legend('Motor DC + Engrenagens + Carga','Funcao Transferencia Motor DC','Modelo em Espaco de Estados');
title('Controlo do tipo PD - Comparacao da Posicao Angular dos tres modelos - Com Filtro');
% Comparacao entre as Velocidades Angulares dos tres modelos
figure
plot(teta_ponto_1.time, teta_ponto_1.signals.values,'r',teta_ponto_2.time, teta_ponto_2.signals.values,'b',x2.time, x2.signals.values,'k');
legend('Motor DC + Engrenagens + Carga','Funcao Transferencia Motor DC','Modelo em Espaco de Estados');
title('Controlo do tipo PD - Comparacao as Velocidades Angulares dos tres modelos - Com Filtro');
%--------------------------------------------------------------------------
% Alinea 5
% 1)
filtro = 0;
Ki = (P*wn^2)/(b0); % A parte integradora num PID serve para anular o erro em regime final
sim('servermotor');
% Motor DC + Engrenagens + Carga
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_1.time, teta_1.signals.values,'r',teta_ponto_1.time, teta_ponto_1.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PID - Motor DC + Engrenagens + Carga - Sem Filtro')
% Funcao Transferencia Motor DC
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_2.time, teta_2.signals.values,'r',teta_ponto_2.time, teta_ponto_2.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PID - Funcao Transferencia Motor DC - Sem Filtro ')
% Espaco de Estados
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',x1.time, x1.signals.values,'r',x2.time, x2.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PID - Modelo em Espaco de Estados - Sem Filtro');
% Comparacao entre as Posic??es Angulares dos tres modelos
figure
plot(teta_1.time, teta_1.signals.values,'r',teta_2.time, teta_2.signals.values,'b',x1.time, x1.signals.values,'k');
legend('Motor DC + Engrenagens + Carga','Funcao Transferencia Motor DC','Modelo em Espaco de Estados');
title('Controlo do tipo PID - Comparacao da Posicao Angular dos tres modelos - Sem Filtro');
% Comparacao entre as Velocidades Angulares dos tres modelos
figure
plot(teta_ponto_1.time, teta_ponto_1.signals.values,'r',teta_ponto_2.time, teta_ponto_2.signals.values,'b',x2.time, x2.signals.values,'k');
legend('Motor DC + Engrenagens + Carga','Funcao Transferencia Motor DC','Modelo em Espaco de Estados');
title('Controlo do tipo PID - Comparacao as Velocidades Angulares dos tres modelos - Sem Filtro');
% 2)
filtro = 100;
sim('servermotor');
% Motor DC + Engrenagens + Carga
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_1.time, teta_1.signals.values,'r',teta_ponto_1.time, teta_ponto_1.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PID - Motor DC + Engrenagens + Carga - Com Filtro')
% Funcao Transferencia Motor DC
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_2.time, teta_2.signals.values,'r',teta_ponto_2.time, teta_ponto_2.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PID - Funcao Transferencia Motor DC - Com Filtro ')
% Espaco de Estados
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',x1.time, x1.signals.values,'r',x2.time, x2.signals.values,'b');
legend('Sinal de Entrada','Posicao Angular','Velocidade Angular');
title('Controlo do tipo PID - Modelo em Espaco de Estados - Com Filtro');
% Comparacao entre as Posic??es Angulares dos tres modelos
figure
plot(teta_1.time, teta_1.signals.values,'r',teta_2.time, teta_2.signals.values,'b',x1.time, x1.signals.values,'k');
legend('Motor DC + Engrenagens + Carga','Funcao Transferencia Motor DC','Modelo em Espaco de Estados');
title('Controlo do tipo PID - Comparacao da Posicao Angular dos tres modelos - Com Filtro');
% Comparacao entre as Velocidades Angulares dos tres modelos
figure
plot(teta_ponto_1.time, teta_ponto_1.signals.values,'r',teta_ponto_2.time, teta_ponto_2.signals.values,'b',x2.time, x2.signals.values,'k');
legend('Motor DC + Engrenagens + Carga','Funcao Transferencia Motor DC','Modelo em Espaco de Estados');
title('Controlo do tipo PID - Comparacao as Velocidades Angulares dos tres modelos - Com Filtro');
%--------------------------------------------------------------------------
%% Alinea 4-5 --> Comportamento PD continuo-discreto e PID continuo-discreto
%Comparacao entre PD e PID aproximado discreto com PD e PID continuo
% PD continuo - Sem Filtro
filtro = 0;
Ki = 0;
sim('servermotor');
teta_PD = teta_1;
% PID continuo - Sem Filtro
filtro = 0;
Ki = (P*wn^2)/(b0); % A parte integradora num PID serve para anular o erro em regime final
sim('servermotor');
teta_PID = teta_1;
sim('Discreto');
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_3.time, teta_3.signals.values,'r-x',teta_4.time, teta_4.signals.values,'g-x',teta_PD.time, teta_PD.signals.values,'b',teta_PID.time, teta_PID.signals.values,'c')
legend('Sinal de Entrada','Posicao Angular - PD discreto','Posicao Angular - PID - discreto','Posicao Angular - PD continuo','Posicao Angular - PID - continuo');
title('Controlo PD e PID - Discreto aproximado e Continuo')
%--------------------------------------------------------------------------
%% Alinea 6
%%Modelo de estado do sistema discreto:
[phi gama] = c2d(A,B,h);
%Observador  preditor  de  estado  aumentado 
phiw = 1;
phixw = gama; 
phi_a = [phi phixw; 0 0 phiw]; 
gama_a = [gama;0]; 
C_a = [C 0] ;
%Matriz de observabilidade 
w0=[C_a; C_a*phi_a; C_a*phi_a^2]; 
w0i=inv(w0);
%Vector de ganhos do observador DEADBEAT
Ko=phi_a^3*w0i*[0; 0; 1] ;
%valor inicial do estado observado 
xobs=[0;0;0]; 
% DEADBEAT
filtro = 100;
Ki = (P*wn^2)/(b0); % A parte integradora num PID serve para anular o erro em regime final
sim('servermotor');
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_1.time, teta_1.signals.values,'b',x1_estimado.time, x1_estimado.signals.values,'r-');
legend('Sinal Entrada','Posicao Angular - PID - Com Filtro','x1 estimado');
title('Comparacao da Posicao Angular com x1 estimado - Observador DEADBEAT');
figure
plot(teta_ponto_1.time,teta_ponto_1.signals.values,'b',x2_estimado.time, x2_estimado.signals.values,'r-');
legend('Velocidade Angular - PID - Com Filtro','x2 estimado');
title('Comparacao da Velocidade Angular com x2 estimado - Observador DEADBEAT');
figure
plot(perturbacao.time, perturbacao.signals.values,'b',p_estimado.time, p_estimado.signals.values,'r-');
legend('Perturbacao aplicada','Perturbacao estimada');
title('Comparacao a perturbacao aplicada e a perturbacao estimada - Observador DEADBEAT');
%--------------------------------------------------------------------------
%% DINAMICA DEFINIDA ATRAV??S DA EQUA????O: (s^2+2*Ksi*wn*s + (wn)^2)*(s+70) = 0
% Queremos a dinamica igual ao do controlador PID da alinea 5.
den=conv([1 2*Ksi*wn (wn)^2], [1 70]); 
[a,b,c,d]=tf2ss([0 0 0 1],den); 
[phio,gamao]=c2d(a,b,h); 
po=eig(phio);
Ko=acker(phi_a',C_a',po); 
Ko=Ko';
xobs=[0;0;0]; 
filtro = 100;
Ki = (P*wn^2)/(b0); % A parte integradora num PID serve para anular o erro em regime final
sim('servermotor');
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_1.time, teta_1.signals.values,'b',x1_estimado.time, x1_estimado.signals.values,'r-');
legend('Sinal Entrada','Posicao Angular - PID - Com Filtro','x1 estimado');
title('Comparacao da Posicao Angular com x1 estimado - Observador com Dinamica dominante de 2?? Ordem');
figure
plot(teta_ponto_1.time, teta_ponto_1.signals.values,'b',x2_estimado.time, x2_estimado.signals.values,'r-');
legend('Velocidade Angular  - PID - Com Filtro','x2 estimado');
title('Comparacao da Velocidade Angular com x2 estimado - Observador com Dinamica dominante de 2?? Ordem');
figure
plot(perturbacao.time, perturbacao.signals.values,'b',p_estimado.time,-p_estimado.signals.values,'r-');
legend('Perturbacao aplicada','Perturbacao estimada');
title('Comparacao a perturbacao aplicada e a perturbacao estimada - Observador com Dinamica dominante de 2?? Ordem');
%--------------------------------------------------------------------------
%% Alinea 7.1
perturbacao_bin_fv = 1;
perturbacao_bin_st = 8;
perturbacao_fv = 0;
perturbacao_st = 0;
% PID
filtro = 100;
Ki = (P*wn^2)/(b0); % A parte integradora num PID serve para anular o erro em regime final
sim('servermotor');
figure
subplot(1,2,1)
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_1.time, teta_1.signals.values,'b',x1_estimado.time, x1_estimado.signals.values,'r-');
legend('Sinal Entrada','Posicao Angular','x1 estimado');
title('PID - Com Filtro - Com perturbacao no Binario');
subplot(1,2,2)
plot(teta_ponto_1.time, teta_ponto_1.signals.values,'b',x2_estimado.time, x2_estimado.signals.values,'r-');
legend('Velocidade Angular','x2 estimado');
title('PID - Com Filtro - Com perturbacao no Binario');
% PD
filtro = 100;
Ki = 0;
sim('servermotor');
figure
title('PD')
subplot(1,2,1)
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_1.time, teta_1.signals.values,'b',x1_estimado.time, x1_estimado.signals.values,'r-');
legend('Sinal Entrada','Posicao Angular','x1 estimado');
title('PD - Com Filtro - Com perturbacao no Binario');
subplot(1,2,2)
plot(teta_ponto_1.time, teta_ponto_1.signals.values,'b',x2_estimado.time, x2_estimado.signals.values,'r-');
legend('Velocidade Angular','x2 estimado');
title('PD - Com Filtro - Com perturbacao no Binario');
%% Alinea 7.2
perturbacao_bin_fv = 0;
perturbacao_bin_st = 0;
perturbacao_fv = 1;
perturbacao_st = 6.5;
% PD
filtro = 100;
Ki = 0;
% Teste 1
rb = 0.02;
mb = 0.01;
Jg = 5.28e-5; %Kg.m2;
JL = Jg + (mb*(rb^2))/2; %Kg.m^2
Jeq = Kg^2*Jm +JL;
sim('servermotor');
% Motor DC + Engrenagens + Carga
teta_PD_1 = teta_1;
% PID
Ki = (P*wn^2)/(b0); % A parte integradora num PID serve para anular o erro em regime final
sim('servermotor');
% Motor DC + Engrenagens + Carga
teta_PID_1 = teta_1;
% Teste 2
% PD
Ki = 0;
rb = 0.01;
mb = 0.07;
Jg = 5.28e-5; %Kg.m2;
JL = Jg + (mb*(rb^2))/2; %Kg.m^2
Jeq = Kg^2*Jm +JL;
sim('servermotor');
% Motor DC + Engrenagens + Carga
teta_PD_2 = teta_1;
% PID
Ki = (P*wn^2)/(b0); % A parte integradora num PID serve para anular o erro em regime final
sim('servermotor');
% Motor DC + Engrenagens + Carga
teta_PID_2 = teta_1;
figure
plot(sinal_entrada.time, sinal_entrada.signals.values,'k',teta_PD_1.time, teta_PD_1.signals.values,'r',teta_PID_1.time, teta_PID_1.signals.values,'b',teta_PD_2.time, teta_PD_2.signals.values,'c--',teta_PID_2.time, teta_PID_2.signals.values,'g--');
legend('Sinal de Entrada','PD --> rb = 0.02 mb = 0.01 ','PID --> rb = 0.02 mb = 0.01','PD --> rb = 0.01 mb = 0.07 ','PID --> rb = 0.01 mb = 0.07');
title('Variacao da Carga - Efeito na Posicao Angular')
%--------------------------------------------------------------------------
%% Alinea 8
Rm=2.6;%ohms
Km=0.0078;%Nm/A
Lm=0.00018;%H
Kgi=14; %relacao entre engrenagens e caixa redutora 
Kge=5; %relacao entre engrenagens do trem externo
Jmrotor=3.9e-7; %kgm^2
Jm=4.60625e-7; %Kg.m^2
JL=6.63e-5; %Kg.m^2
Jg=5.28e-5; %Kg.m2;
Kg=Kge*Kgi;
Jeq= Kg^2*Jm +JL;
Jl=JL;
Kf=0;
Ke=Km;
Bl=0;
perturbacao_bin_st = 0;
perturbacao_bin_fv = 0;
perturbacao_fv = 1;
perturbacao_st = 20;
A = [0 1 ; 0 -0.5460/0.0111];
B = [0 1/0.0111]';
C = [1 0];
D = 0;
h=0.01;
[phi,gama]=c2d(A,B,h); 
%Controlador: definicao do polinomio caracteristico 
zeta=0.7;
wn=21.9955; 
den=[1 2*zeta*wn wn*wn]; 
[a,b,c,d]=tf2ss([0 0 1], den); 
[phi_cl,gama_cl]=c2d(a,b,h); 
p_cl=eig(phi_cl); 
L=acker(phi, gama, p_cl); 
Lw=1; 
La=[L Lw]; 
%Estado inicial 
x0=[1;1]; 
%Ganho Lc para seguimento com ganho DC unitario 
phic=phi-gama*L; 
Lc=1/(C*inv(eye(2)-phic)*gama);
%Observador  preditor  de  estado  aumentado
phiw=1; phixw=gama; 
phi_a_2=[phi phixw; 0 0 phiw]; 
gama_a_2=[gama;0]; 
C_a_2=[C 0]; 
%Matriz de observabilidade 
w0=[C_a_2; C_a_2*phi_a_2; C_a_2*phi_a_2^2];
w0i=inv(w0); 
%Vector de ganhos do observador deadbeat 
Ko_2=phi_a_2^3*w0i*[0; 0; 1]; 
%valor inicial do estado observado 
xobs_2=[0;0;0]; 
sim('servermotor') % chamada o modelo Simulink 
figure
plot(sinal_entrada.time,sinal_entrada.signals.values,'c',x1_est.time, x1_est.signals.values,'b:',x2_est.time, x2_est.signals.values,'r',p_est.time, p_est.signals.values,'g:'); 
legend('Sinal de Entrada','x1 - Posicao Angular','x2 - Velocidade Angular','Perturbacao Estimada')
title('Realimentacao dos Estados Estimados com perturbacao na entrada');
axis([ 0 30 -1 1.5]);
perturbacao_bin_st = 20;
perturbacao_bin_fv = 1;
perturbacao_fv = 0;
perturbacao_st = 0;
sim('servermotor') % chamada o modelo Simulink 
figure
plot(sinal_entrada.time,sinal_entrada.signals.values,'c',x1_est.time, x1_est.signals.values,'b:',x2_est.time, x2_est.signals.values,'r',p_est.time, p_est.signals.values,'g:'); 
legend('Sinal de Entrada','x1 - Posicao Angular','x2 - Velocidade Angular','Perturbacao Estimada')
title('Realimentacao dos Estados Estimados com perturbacao no binario');