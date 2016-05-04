function estim=obx(ent) 
global xobs Ko C_a phi_a gama_a; 
uk1=ent(1); 
yok=ent(2); 
%predicao do estado aumentado 
xobs=phi_a*xobs+gama_a*uk1+Ko*(yok-C_a*xobs); 
%Devolucao do estado observado 
estim=xobs; 