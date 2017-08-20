function [v,w,t] = GetShortestPath(q1,q2,r)
%
%   start and goal configurations:
%       q1 = [x;y;theta]
%       q2 = [x;y;theta]
%
%   minimum turning radius:
%       r
%
%   shortest path (all 1x3 matrices):
%       v is forward speed (normalized - always 1 or -1)
%       w is turning rate
%       t is time interval (i.e., apply v(i) and w(i) for time t(i))
%
%   if no path is found, [v,w,t] will all be empty sets
%
%   note that the total length of the shortest path is sum(t)
%

d1=inf;
dmid=inf;
d2=inf;
s1 = [];
dir = [];
for s1cur=[-1 1]
    for s2cur=[-1 1]
        for dircur=[-1 1]
            [d1cur,dmidcur,d2cur,qt1cur,qt2cur]=twb_shortpath(q1,s1cur,q2,s2cur,dircur,r);
            if (~isnan(d1))
                if ((d1cur+dmidcur+d2cur)<(d1+dmid+d2))
                    d1=d1cur;
                    dmid=dmidcur;
                    d2=d2cur;
                    s1=s1cur;
                    s2=s2cur;
                    dir=dircur;
                    qt1=qt1cur;
                    qt2=qt2cur;
                end
            end
        end
    end
end

if (isnan(d1)) || isempty(s1) || isempty(dir)
    v = [];
    w = [];
    t = [];
    return;
else
    v = [dir dir dir];
    w = [s1*v(1)/r 0 s2*v(3)/r];
    t = [d1 dmid d2];
end

function [d1,dmid,d2,qt1,qt2]=twb_shortpath(q1,s1,q2,s2,dir,r)
% function [d1,dmid,d2,qt1,qt2]=twb_shortpath(q1,s1,q2,s2,dir,r)
%
%   q1,q2 are initial and final points
%   s1,s2 are -1 for wheels to left, +1 for wheels to right
%   dir is 1 for forward and -1 for backward
%   r is turning radius
%
%   d1,dmid,d2 are lengths of each part of the curve in R^2
%   qt1,qt2 are intermediate points
%
%   NOTE: particular combinations of dir, s1, s2 are sometimes infeasible.
%   If this is the case, all output variables will be set to NaN. To check
%   for this, call isnan.
%

h1=q1(3);
h2=q2(3);
q1=q1(1:2);
q2=q2(1:2);
qc1=q1+r*s1*[-sin(h1);cos(h1)];
qc2=q2+r*s2*[-sin(h2);cos(h2)];

% Check for infeasibility...
if ((s1*s2)<0)
    if (norm(qc2-qc1)<(2*r))
        d1=NaN;
        dmid=NaN;
        d2=NaN;
        qt1=NaN;
        qt2=NaN;
        return;
    end
end

[qt1,qt2]=twb_gettangent(qc1,dir*s1,qc2,dir*s2,r);
delta1=twb_getangle(qc1,qt1)-twb_getangle(qc1,q1);
if ((dir*s1>0)&(delta1<0))
    delta1=delta1+2*pi;
elseif (((dir*s1)<0)&(delta1>0))
    delta1=delta1-2*pi;
end
delta2=twb_getangle(qc2,q2)-twb_getangle(qc2,qt2);
if ((dir*s2>0)&(delta2<0))
    delta2=delta2+2*pi;
elseif (((dir*s2)<0)&(delta2>0))
    delta2=delta2-2*pi;
end
d1=abs(r*delta1);
dmid=norm(qt2-qt1);
d2=abs(r*delta2);
qt1=[qt1;h1+delta1];
qt2=[qt2;h2-delta2];

function [t1,t2]=twb_gettangent(q1,s1,q2,s2,r)
% function [t1,t2]=twb_gettangent(q1,s1,q2,s2,r)
%
% q1,q2 are the centers of two circles, both with radius r
% s1,s2 are the orientations (1 or -1) of the two circles
%		(1=cc-wise, -1=c-wise)
%
% t1,t2 are the endpoints of a segment that is tangent to
%		both circles and that matches the orientations s1,s2
%
% NOTE: q1,q2 must be column vectors.

if ((s1==1)&(s2==1))
	% Vector u from center 1 to center 2
	u=q2-q1;
	% Vector perpendicular to u (turning clockwise) of length r
	uperp=-r*[-u(2); u(1)]/norm(u);
	% Tangent points
	t1=q1+uperp;
	t2=q2+uperp;
elseif ((s1==-1)&(s2==-1))
	% Vector u from center 1 to center 2
	u=q2-q1;
	% Vector perpendicular to u (turning counter-clockwise) of length r
	uperp=r*[-u(2); u(1)]/norm(u);
	% Tangent points
	t1=q1+uperp;
	t2=q2+uperp;
elseif ((s1==1)&(s2==-1))
	% Vector to midpoint of segment from center 1 to center 2
	qmid=((q2-q1)/2);
	% Vector to tangent points in local coordinates
	L=norm(qmid);
	x=(r^2)/L;
	y=(r/L)*sqrt((L^2)-(r^2));
	% Unit vector in the direction of qmid, and its perpendicular
	qmid=qmid/norm(qmid);
	qmidperp=[-qmid(2); qmid(1)];
	% Tangent points
	t1=q1+x*qmid-y*qmidperp;
	t2=q2-x*qmid+y*qmidperp;
elseif ((s1==-1)&(s2==1))
	% Vector to midpoint of segment from center 1 to center 2
	qmid=((q2-q1)/2);
	% Vector to tangent points in local coordinates
	L=norm(qmid);
	x=(r^2)/L;
	y=(r/L)*sqrt((L^2)-(r^2));
	% Unit vector in the direction of qmid, and its perpendicular
	qmid=qmid/norm(qmid);
	qmidperp=[-qmid(2); qmid(1)];
	% Tangent points
	t1=q1+x*qmid+y*qmidperp;
	t2=q2-x*qmid-y*qmidperp;
else
	error(sprintf('twb_gettangent was passed s1=%f, s2=%f',s1,s2));
end

function h=twb_getangle(q1,q2)
h=atan2(q2(2)-q1(2),q2(1)-q1(1));



