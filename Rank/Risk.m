function risk=Risk(risk_score, n)
riskPercent = risk_score / n;  % n is the number of the parkingslot data
if riskPercent <= 0.2
    risk=10;
elseif riskPercent <= 0.4
    risk=6;
elseif riskPercent <= 0.6
    risk=4;
elseif riskPercent <= 0.8
    risk=2;
else
    risk=1;
end