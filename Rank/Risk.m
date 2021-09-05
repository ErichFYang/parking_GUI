function risk=Risk(risk_score)
if risk_score<=500
    risk=10;
elseif risk_score<=1000
    risk=6;
elseif risk_score<=1500
    risk=4;
elseif risk_score<=2000
    risk=2;
else
    risk=1;
end