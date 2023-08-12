%% Sintonía de Control PID via COHEN y COON
% Sergio Andres Castaño Giraldo
% https://controlautomaticoeducacion.com/

%% Proceso Real
G=tf(Gf);
G.iodelay=5;
[num,den] = tfdata(G,'v');
L = G.iodelay;
%% Identificacion del Modelo via Ziegler y Nichols
%tiempo
dt=0.01;
t=0:dt:35;
%Entrada
u(1:length(t))=1;
%Salida
y=lsim(G,u,t);
dy=diff(y)/dt; %Derivada
dy2=diff(dy)/dt;

%Encontrar el punto de inflexion y su derivada
% el punto donde la pendiente de la respuesta escalón tiene su valor máximo (punto de inflexión)
[m,p]=max(dy);
yp=y(p);
tp=t(p);
tm=0:20; 
ym=m*(tm-tp)+yp; %Ecuacion de la recta

%Grafica punto de inflexión (Meramente ilustrativa)
figure
plot([t(1) t(end)],[2 2],'--k',[15 15],[0 2],'--k','linewidth',2);
hold on
plot(t,y,tp,yp,'o',tm,ym,'-r','linewidth',3);
axis([0 35 0 3]); 
box off
ylabel('$$c(t)$$','FontSize',20,'Interpreter','latex')
xlabel('$$t$$','FontSize',20,'Interpreter','latex')
set(gca,'FontSize',(20) )

%% Modelo Identificado del Sistema
k = 2;    %Ganancia del Sistema
theta=6;  %Retardo
tau=3; %Constante de Tiempo
%Modelo
Gm=tf([k],[tau 1]);
Gm.iodelay=theta;

fprintf("El factor de Incontrolabilidad es: %f\n",theta/tau)

%% Comparar Sistema Real vs Modelo
figure
step(G,Gm)
legend('Planta','Modelo')

%% Método COHEN Y COON
% Control P - COHEN Y COON
kc1 = (1.03 + 0.35 * theta/tau) * tau / (k*theta);
%Ganancias
Kp1 = kc1;

% Control PI - COHEN Y COON
kc2 = (0.9 + 0.083 * theta/tau) * tau / (k*theta);
ti2 = theta * (0.9 + 0.083 * theta/tau) / (1.27 + 0.6 * theta/tau);
%Ganancias
Kp2 = kc2;
Ki2 = kc2/ti2;

% Control PID - COHEN Y COON
kc3 = (1.35 + 0.25 * theta/tau) * tau / (k*theta);
ti3 = theta * (1.35 + 0.25 * theta/tau) / (0.54 + 0.33 * theta/tau);
td3 = (0.5 * theta) / (1.35 + 0.25 * theta/tau);
%Ganancias
Kp3 = kc3;
Ki3 = kc3/ti3;
Kd3 = kc3*td3;