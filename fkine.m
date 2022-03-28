function T = fkine(S,M,q)
    len = height(S');
        T = eye(4);
        for i = 1:len
            T = T*twist2ht(S(:,i),q(i));
        end
        T = T*M;
end