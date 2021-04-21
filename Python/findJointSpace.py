from ece470_lib import *

a0 = [0,0,1];
a1 = [0,1,0];
a2 = [0,1,0];
a3 = [0,1,0];
a4 = [1,0,0];

inTOmm = 25.4;

q0 = [0,0,0];
q1 = [3.76*inTOmm,0,8.11*inTOmm];
q2 = [11.76*inTOmm,0,8.11*inTOmm];
q3 = [17.76*inTOmm,0,8.11*inTOmm];
q4 = [19.48*inTOmm,0,8.11*inTOmm];

s0 = toScrew(a0,q0);
s1 = toScrew(a1,q1);
s2 = toScrew(a2,q2);
s3 = toScrew(a3,q3);
s4 = toScrew(a4,q4);
S=[s0,s1,s2,s3,s4];

toolRot = np.array([[ 0,0,1],
                    [ 0,1,0],
                    [-1,0,0]]);
toolPos = np.array([[19.48*inTOmm+600 , 0 , 8.11*inTOmm]]).transpose();
M=toPose(toolRot,toolPos);

goalRot = np.eye(3);
X=np.arange(-10,11,0.5);
Y=X;
Z=np.arange(600-10,600+11,0.5)
thetaMap = [[[np.ones((5,1))*-99 for i in range(Z.size)] for j in range(Y.size)] for k in range(X.size)];
i=0;
for x in range(X.size):
    for y in range(Y.size):
        for z in range(Z.size):
            print(i);
            i+=1;
            goalPos = np.array([[0,0,600+15*inTOmm]]).transpose();
            goalT = toPose(goalRot,goalPos);
            theta,norm = findIK(goalT, S, M);
            if(norm<0.01):
                thetaMap[x][y][z]=theta;
print(theta);
print(norm);
