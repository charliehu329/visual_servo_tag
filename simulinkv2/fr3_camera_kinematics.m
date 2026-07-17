function [pWCL,RWCL,pWCR,RWCR,JL,JR,rankJL,rcondJL,rotationOrthogonalityError,baselineLength,baselineError,stereoRotationError,pWLink8,RWLink8] = fr3_camera_kinematics(q,cfg)
%FR3_CAMERA_KINEMATICS Fixed-size FR3 FK and camera optical-center Jacobians.
% Runtime computation is independent of rigidBodyTree and uses URDF-derived
% parameters already stored in cfg by init_arm_stereo_ibvs_ekf_v1.
%#codegen

pWCL=zeros(3,1); RWCL=eye(3); pWCR=zeros(3,1); RWCR=eye(3);
JL=zeros(6,7); JR=zeros(6,7); rankJL=0; rcondJL=0;
rotationOrthogonalityError=0; baselineLength=0; baselineError=0;
stereoRotationError=0; pWLink8=zeros(3,1); RWLink8=eye(3);

T=cfg.T_W_B;
jointOrigins=zeros(3,7); jointAxes=zeros(3,7);
for i=1:7
    T=T*makeTransform(cfg.fr3OriginXYZ(:,i),cfg.fr3OriginRPY(:,i));
    jointOrigins(:,i)=T(1:3,4);
    jointAxes(:,i)=T(1:3,1:3)*cfg.fr3Axis(:,i);
    T=T*axisRotation(cfg.fr3Axis(:,i),q(i));
end
T=T*makeTransform(cfg.fr3Joint8OriginXYZ,cfg.fr3Joint8OriginRPY);
pWLink8=T(1:3,4); RWLink8=T(1:3,1:3);
TCL=T*cfg.T_link8_CL;
pWCL=TCL(1:3,4); RWCL=TCL(1:3,1:3);
TCR=TCL*cfg.T_CL_CR;
pWCR=TCR(1:3,4); RWCR=TCR(1:3,1:3);

JW=zeros(6,7);
for i=1:7
    JW(1:3,i)=cross(jointAxes(:,i),pWCL-jointOrigins(:,i));
    JW(4:6,i)=jointAxes(:,i);
end
RCLW=RWCL';
JL=[RCLW zeros(3,3);zeros(3,3) RCLW]*JW;
R=cfg.R_CL_CR; p=cfg.p_CL_CR;
JR=[R' -R'*skew3(p);zeros(3,3) R']*JL;
rankJL=rank(JL);
rcondJL=rcond(JL*JL');
rotationOrthogonalityError=norm(RWCL'*RWCL-eye(3),'fro');
baselineLength=norm(pWCR-pWCL);
baselineError=abs(baselineLength-norm(cfg.p_CL_CR));
stereoRotationError=norm(RWCR-RWCL*cfg.R_CL_CR,'fro');
end

function T=makeTransform(xyz,rpy)
T=eye(4);
T(1:3,1:3)=rotz3(rpy(3))*roty3(rpy(2))*rotx3(rpy(1));
T(1:3,4)=xyz;
end
function T=axisRotation(axis,q)
u=axis/max(norm(axis),eps);
K=skew3(u); R=eye(3)+sin(q)*K+(1-cos(q))*(K*K);
T=eye(4); T(1:3,1:3)=R;
end
function R=rotx3(a), c=cos(a);s=sin(a);R=[1 0 0;0 c -s;0 s c]; end
function R=roty3(a), c=cos(a);s=sin(a);R=[c 0 s;0 1 0;-s 0 c]; end
function R=rotz3(a), c=cos(a);s=sin(a);R=[c -s 0;s c 0;0 0 1]; end
function S=skew3(v), S=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0]; end
