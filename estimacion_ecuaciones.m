P1 = zpk([0.19, 0], [-1.0135 + 15.2396i, -1.0135 - 15.2396i, -30.5063 + 91.3426i, -30.5063 - 91.3426i], 12062);

sys = ss(P1);

[A, B , C ,D] = ssdata(sys);


controlabilidad = [B A*B A*A*B A*A*A*B];
observabilidad = [C ; C*A ; C*A*A ; C*A*A*A];


syms x1 x2 x3 x4 u real
x = [x1; x2; x3;x4];

dx = A*x + B*u;
y = C*x + D*u;

dx_simplificado = simplify(dx);
y_simplificado = simplify(y);



%% -----------------diseño observador------------------------------------
polos_controlador = [-10+ 11.2i, -10 - 11.2i ,-30.5063 + 91.3426i, -30.5063 - 91.3426i];

polos_observador = [-120 , -130,-140 ,-150];


L = place(A', C', polos_observador)';
T = P1/(1+P1);

%polos en 50 para respuesta rapida

Kc = acker(A,B,polos_observador);

Lc = acker(A.',C.',polos_observador).';
% codigo vampus
x0=[0 0 90 0];

Boc =[B Lc];
Aoc =[A - Lc*C];
Chat=eye(length(A));
Dhat= zeros (length(A));
%Do =[0 0;0 0;0 0;0 0];
Do = 0;
Bhat =eye(length(A));
observadorc=ss(Aoc,Boc,Chat,Do);%observador continuo
[Aocc,Bocc,Cocc,Docc]= ssdata(observadorc);
obs_tf = ss(Aocc,Bocc,Cocc,Docc);
obs_tf_tf = tf(obs_tf(1,:)); %Tf u a y

%cosas del libro
%u = -K*x + kff*r.

%Tcl = obs_tf_tf/(1+obs_tf_tf)
%figure;
%step(Tcl);


%% Ley de control calculo de K continuo y discreto
% polinomio deseado controlador en continuo mantengo wn y bajo zeda de la
% planta
% polinomio deseado controlador en discreto z=exp(sTs) pic  pi

P1 = zpk([0.19, 0], [-1.0135 + 15.2396i, -1.0135 - 15.2396i, -30.5063 + 91.3426i, -30.5063 - 91.3426i], 12062);
sys = ss(P1);
[A, B , C ,D] = ssdata(sys);

controlabilidad = [B A*B A*A*B A*A*A*B];
observabilidad = [C ; C*A ; C*A*A ; C*A*A*A];

Ts = 0.01; % tiempo de muestreo en segundos

sysd = c2d(ss(A, B, C, D), Ts);

[Ad, Bd, Cd, Dd] = ssdata(sysd);

Ts = 0.01;
p1c=-10+ 11.2i ;p2c=-10- 11.2i; p3c=-30.5063 + 91.3426i; p4c=-30.5063 + 91.3426i;
p1= exp(p1c*Ts) ;p2= exp(p2c*Ts) ; p3= exp(p3c*Ts) ;p4= exp(p4c*Ts) ;
polyc =[p1c ;p2c;p3c;p4c] ;%polos deseados continup
polyd =[p1 ;p2;p3;p4] ;%polos deseados discretos
Kc = acker(A,B,polyc);
Kd = acker(Ad,Bd,polyd);

% polinomio del estimador LCe y en discreto
% polinomio deseado estimador en continuo poly = (s+50)^4  z=exp(sTs)
pec = -60 ;
ped = -60 ;
pe= exp(ped*Ts) ;
polyec =[pec pec pec pec] ;
Lc = acker(A.',C.',polyec).';
polyed =[pe pe pe pe] ;
Ld = acker(Ad.',Cd.',polyed).';
x0=[0 0 90 0];
%
% Armado del observador
%
Boc =[B Lc];
Aoc =[A - Lc*C];
Bo =[Bd Ld];
Ao =[Ad - Ld*Cd];
Chat=eye(length(A));
Dhat= zeros (length(A));
Do =[0 0;0 0;0 0;0 0];
Bhat =eye(length(A));
observadorc=ss(Aoc,Boc,Chat,Do);%observador continuo
[Aocc,Bocc,Cocc,Docc]= ssdata(observadorc);
observadord=ss(Ao,Bo,Chat,Do);%observador discreto

%%
s = tf('s');
C = 0.79 * (s+10)/s;
L = P1*C;
T = L/(1+L);

poles = pole(T);

disp(poles);%% -30 + 133j, -3 + 10j




