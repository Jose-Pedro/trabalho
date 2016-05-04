function estim=obx(ent) 
global xobs_2 Ko_2 C_a_2 phi_a_2 gama_a_2; 
uk1=ent(1); 
yok=ent(2); 
%predicao do estado aumentado 
xobs_2=phi_a_2*xobs_2+gama_a_2*uk1+Ko_2*(yok-C_a_2*xobs_2); 
%Devolucao do estado observado 
estim=xobs_2; 