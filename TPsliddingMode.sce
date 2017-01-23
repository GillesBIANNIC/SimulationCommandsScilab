xdel(winsid());
clear;
clf;
funcprot(0);//supprime un avertissement génant à l'execution

function theChar = draw_char(x)
    //drawlater();
    // 1) Configuration du graphique
    clf();   // efface la figure courante 
    set(gca(),"auto_clear","off") //hold on; en matlab // superposition des courbes
    mtlb_axis('off'); // pour effacer les axes
    mtlb_axis([-20,20,-20,20]);   // repère
    // 2) Dessin de l'objet simulé
    C = [-1.5,0,0,-0.5,0.5,0,0,1.5, 2,2 ,1.5 ,0,0,-0.5,0.5,0,0,-1.5,-1.5;
          1,1,1.5,1.5,1.5,1.5,1,1,0.5,-0.5,-1,-1,-1.5,-1.5,-1.5,-1.5,-1,-1, 1;
          1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
          1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1];
    // 3) Matrice de transformation homogène, rotation selon z et translation dans le plan (x,y)
    matRot=[cos(x(3)) -sin(x(3)) 0 x(1);
            sin(x(3))  cos(x(3)) 0 x(2);
               0         0       1   0;
               0         0       0   1];
    // 4) Application de la transformation homogène
    theChar=matRot*C;
    // 5) Dessin du motif transformé
    //plot([C(1,:)], [C(2,:)],charColor,'LineWidth',2);
    //drawnow(); //pairé avec drawlater(), permet d'éviter l'effet de clignotement désagréable
endfunction

function [tricC,tricR] = draw_tric(x)
    // 1) Configuration du graphique
    clf();   // efface la figure courante 
    set(gca(),"auto_clear","off") //hold on; en matlab // superposition des courbes
    mtlb_axis('off'); // pour effacer les axes
    mtlb_axis([-10,10,-10,10]);   // repère
    // 2) Dessin de l'objet simulé
    // char
    C = [-1.5,0,0,-0.5,0.5,0,0,1.5, 2,2 ,1.5 ,0,0,-0.5,0.5,0,0,-1.5,-1.5;
          1,1,1.5,1.5,1.5,1.5,1,1,0.5,-0.5,-1,-1,-1.5,-1.5,-1.5,-1.5,-1,-1, 1;
          1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
          1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1];
    // roue
    R = [-0.5,0.5,0.5,-0.5,-0.5;
          0.2,0.2,-0.2,-0.2, 0.2;
          1,1,1,1,1;
          1,1,1,1,1];
    // 3) Matrice de transformation homogène, rotation selon z et translation dans le plan (x,y)
    matRot=[cos(x(3)) -sin(x(3)) 0 x(1);
            sin(x(3))  cos(x(3)) 0 x(2);
               0         0       1   0;
               0         0       0   1];
    matRotRoue=[cos(x(5)) -sin(x(5)) 0 2.8; //+2.8 in x
                sin(x(5))  cos(x(5)) 0 0;
                   0         0       1   0;
                   0         0       0   1];
    // 4) Application de la transformation homogène
    tricC=matRot*C;
    tricR=matRot*matRotRoue*R;
    // 5) Dessin du motif transformé
    //plot([C(1,:)], [C(2,:)],'black','LineWidth',2);
    //plot([R(1,:)], [R(2,:)],'blue','LineWidth',3);
endfunction

function dotx=evolution_char(x,u)
    dotx = [x(4)*cos(x(3));
            x(4)*sin(x(3));
            u(1);
            u(2)];
endfunction

function dotx=evolution_tric(x,u)
    dotx = [x(4)*cos(x(5))*cos(x(3));
            x(4)*cos(x(5))*sin(x(3));
            x(4)*sin(x(5)); // L = 1
            u(1);
            u(2)];
endfunction

function u = slidingRegulation(x, point)
    K = 10;
    Ax = [ -x(4)*sin(x(3)) cos(x(3));
            x(4)*cos(x(3)) sin(x(3))];
    v = [ K * sign( point(1) - x(1) + 0 - x(4)*cos(x(3)) );
          K * sign( point(2) - x(2) + 0 - x(4)*sin(x(3)) )];
    u = inv(Ax)*v;
endfunction

function drawAll( voiture, roueVoiture, remorque1, remorque2, trajectoireX, trajectoireY )
    // draw
    plot([voiture(1,:)], [voiture(2,:)],'black','LineWidth',2);
    plot([roueVoiture(1,:)], [roueVoiture(2,:)],'black','LineWidth',3);
    plot([remorque1(1,:)], [remorque1(2,:)],'blue','LineWidth',2);
    plot([remorque2(1,:)], [remorque2(2,:)],'red','LineWidth',2);
    // trajectory
    plot(trajectoireX, trajectoireY);
endfunction

// commande (trajectoire)
R=10; f1=0.1;
dt=0.02;
t = 0:dt:100;
trajX = R*cos(f1*t);
trajY = R*sin(3*f1*t);

// voiture (tricycle)
x=[0;0;0;0;%pi/12];// vecteur d'état
u=[0,0];
[tricC, tricR] = draw_tric(x);
Err = [];

// remorque 1
c1x=[3,-4,%pi/6.0,0.2]'; // vecteur d'état char 1
c1u=[1;1];
char1 = draw_char(c1x);
Err1 = [];

// remorque 2
c2x=[1,-4,%pi/6.0,0.2]'; // vecteur d'état char 1
c2u=[1;1];
char2 = draw_char(c2x);
Err2 = [];

L1 = 3;
L2 = 3;

for i=0:dt:25
    drawlater();
    // maj commande
    xd = R*cos(f1*i);
    yd = R*sin(3*f1*i);
    dxd = -R*f1*sin(f1*i);
    dyd = R*3*f1*cos(3*f1*i);
    ddxd = -R*f1*f1*cos(f1*i);
    ddyd = -R*9*f1*f1*sin(3*f1*i);
    
    // tricycle
    v = [ xd - x(1) + 2*(dxd - x(4)*cos(x(3))) + ddxd ;
           yd - x(2) + 2*(dyd - x(4)*sin(x(3))) + ddyd ];
    Ax = [ cos(x(3)+x(5)) -x(4)*sin(x(3)+x(5)) ;
           sin(x(3)+x(5)) x(4)*cos(x(3)+x(5)) ];
    Bx = [ x(4)*x(4) * sin(x(5)) * (-sin(x(3)+x(5))) ;
           x(4)*x(4) * sin(x(5)) * ( cos(x(3)+x(5))) ];
    u = Ax\(v-Bx);
    x = x + evolution_tric(x,u) * dt;
    [tricC, tricR] = draw_tric(x);
    Err = [Err ; abs(xd - x(1)) + abs(yd - x(2))];
    
    // remorque 1 vise le point derrière le tricycle d'une distance L
    c1u = slidingRegulation(c1x, [x(1)-L1*cos(x(3)), x(2)-L1*sin(x(3))]);
    c1x = c1x + evolution_char(c1x,c1u) * dt;
    char1 = draw_char(c1x);
    Err1 = [Err1 ; abs(x(1)-L1*cos(x(3))-c1x(1)) + abs(x(2)-L1*sin(x(3))-c1x(2))];

    // remorque 2 vise le point derrière la premiere remorque
    c2u = slidingRegulation(c2x, [c1x(1)-L2*cos(c1x(3)), c1x(2)-L2*sin(c1x(3))]);
    c2x = c2x + evolution_char(c2x,c2u) * dt;
    char2 = draw_char(c2x);
    Err2 = [Err2 ; abs(c1x(1)-L2*cos(c1x(3))-c2x(1)) + abs(c1x(2)-L2*sin(c1x(3))-c2x(2))];
    
    // draw
    drawAll(tricC, tricR, char1, char2, trajX, trajY);
    plot(x(1)-L1*cos(x(3)), x(2)-L1*sin(x(3)), '*'); // debug (pour voir l'attache-remorque du tricycle)
    plot(c1x(1)-L1*cos(c1x(3)), c1x(2)-L1*sin(c1x(3)), '+'); // debug (pour voir l'attache-remorque de la remorque 1)
    plot(xd,yd,'Ok'); // debug du point visé
    drawnow();
end

figure();
plot(Err);
figure();
plot(Err1);
figure();
plot(Err2);
