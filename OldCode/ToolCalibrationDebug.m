
function ToolCalibrationDebug
    Mr1 = createHy(-pi/6,1)
    Mr2 = createHy(pi/6,-1)
    Mr3 = createHx(pi/6,1)
    II = -eye(3)
    A = [Mr1(1:3,1:3) II;Mr2(1:3,1:3) II;Mr3(1:3,1:3) -eye(3)]
    b = -[Mr1(1:3,4);Mr2(1:3,4);Mr3(1:3,4)]
    X = A\b
    A*X
    
    Mxyz = X(1:3)
    xyz0 = X(4:6)
    
    Mtool = eye(4);
    Mtool(1,4) = Mxyz(1);
    Mtool(2,4) = Mxyz(2);
    Mtool(3,4) = Mxyz(3);
    
    Mr1*Mtool*[0 0 0 1]'
    Mr2*Mtool*[0 0 0 1]'
    Mr3*Mtool*[0 0 0 1]'

A=[ -0.900299   0.235628   -0.36598         -1          0          0;
-0.0870015   0.726426   0.681715          0         -1          0;
  0.426489   0.645588  -0.633501          0          0         -1;
 -0.974324   0.121874  -0.189312         -1          0          0;
 0.0450945   0.929423   0.366251          0         -1          0;
  0.220587    0.34831  -0.911055          0          0         -1;
 -0.968952   0.116522    0.21807         -1          0          0;
   0.10408    0.99226 -0.0677342          0         -1          0;
 -0.224274 -0.0429344   -0.97358          0          0         -1];

b=[  0.397215 ;-0.0431174; -0.359472; 0.464843;-0.163925; -0.420984;0.598478;-0.311577; -0.398507];
X=[-0.0334967;-0.0717622;0.324918;-0.502957; 0.214999; 0.09279]
A*X

Mtool = [0.9999288131  0  0.003829768729  -0.03349673565;
        0  1  -0.01130051761  -0.07176219852;
        -0.003829768729  0  0.9999288131  0.3249177252;
        0  0  0  1]
    
 
end

function H = createHz(theta,dx)
    H = [cos(theta) -sin(theta) 0 dx; sin(theta) cos(theta) 0 0; 0 0 1 1;0 0 0 1];
end


function H = createHy(theta,dx)
    H = [cos(theta) 0 sin(theta) dx; 0 1 0 0; -sin(theta) 0 cos(theta) 1;0 0 0 1];
end

function H = createHx(theta,dx)
    H = [1 0 0 0;0 cos(theta) -sin(theta) dx; 0 sin(theta) cos(theta) 1; 0 0 0 1];
end

