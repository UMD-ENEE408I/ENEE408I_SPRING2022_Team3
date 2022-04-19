phi = 0;
c = [0; 0];
m_curr = [10 20];
m_prev = [0 0];
figure(1);
scatter(c(1), c(2));
hold on;
%just random incrememnts for testing purposes
for i = 1:100
    rinc = [randi([10 25]), randi([10 25])];
    if rinc(1) == rinc(2) %I actually need to implement some form of edge casing or else I'll get Naan values
        rinc(1) = rinc(1) + 1;
    end
    n = adjCm(m_curr, m_prev);
    [r, rcoord, theta, phi] = calculateRad(n(1), n(2), c, phi);
    c_prev = c;
    c = newC(theta, c, rcoord)
    scatter(c(1), c(2));
    line([c_prev(1) c(1)], [c_prev(2) c(2)]);
    m_prev = m_curr;
    m_curr = m_curr + rinc;
end
hold off;

function c = newC(theta, c, rcoord)
    c = [cosd(theta) -sind(theta); sind(theta) cosd(theta)]*(c - rcoord) + rcoord;
end

function n = adjCm(m_curr, m_prev)
    % Circumference of the wheel is 11 cm, and each full rotation is 360
    % ticks, so 1 tick is 11/360 cm. 
    n = (m_curr-m_prev)*(11/360);
end

function [r, rcoord, theta, phi] = calculateRad(n1, n2, c, phi)
    if(n1 < n2) %left turn
        a = -1;
        smlTurn = n1;
    else
        a = 1;
        smlTurn = n2;
    end
    
    r = 4*a*(n1 + n2)/(n1 - n2);
    rcoord = c + r*[cosd(phi-a*90); sind(phi-a*90)];
    theta = calculateTurn(smlTurn, r);
    phi = phi - a*theta;    
end

function [theta] = calculateTurn(nx, r)
    theta = 180*nx/(2*pi*(r-4));
end
