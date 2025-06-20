clear all;
clc;

%% Ley de control calculo de K continuo y discreto
P1 = zpk([0.19, 0], [-1.0135 + 15.2396i, -1.0135 - 15.2396i, -30.5063 + 91.3426i, -30.5063 - 91.3426i], 12062);

sys = ss(P1);

[A, B , C ,D] = ssdata(sys);

controlabilidad = [B A*B A*A*B A*A*A*B];
observabilidad = [C ; C*A ; C*A*A ; C*A*A*A];

s = tf('s');
C1 = 0.79 * (s+10)/s;
L = P1*C1;
T = L/(1+L);
Ts=0.02;
poles = pole(T);

Ad = eye(size(A)) + Ts * A;
Bd = Ts * B;
Cd = C;
% polinomio deseado controlador en continuo mantengo wn y bajo zeda de la
% planta-30 + 133j, -3 + 10j
% polinomio deseado controlador en discreto z=exp(sTs) pic  pi

p1c=-3+i*10 ;p2c=-3-i*10; p3c=-10+i*30 ; p4c=-10-i*30;
p1= exp(p1c*Ts) ;p2= exp(p2c*Ts) ; p3= exp(p3c*Ts) ;p4= exp(p4c*Ts) ;
polyc =[p1c ;p2c;p3c;p4c] ;%polos deseados continup
polyd =[p1 ;p2;p3;p4] ;%polos deseados discretos
Kc = acker(A,B,polyc);
Kd = acker(Ad,Bd,polyd);

%% polinomio del estimador LCe y en discreto
%% polinomio deseado estimador en continuo poly = (s+50)^4  z=exp(sTs)
pec = -70 ;
ped = -70 ;
pe= exp(ped*Ts) ;
polyec =[pec pec pec pec] ;
Lc = acker(A.',C.',polyec).';
polyed =[pe pe pe pe] ;
Ld = acker(Ad.',Cd.',polyed).';
%%
%% Armado del observador
%%
Boc =[B Lc];
Aoc =[A - Lc*C];
Do =[0 0;0 0;0 0;0 0];
Bhat =eye(length(A));
observadorc=ss(Aoc,Boc,Chat,Do);%observador continuo
[Aocc,Bocc,Cocc,Docc]= ssdata(observadorc);
observadord=ss(Ao,Bo,Chat,Do);%observador discreto