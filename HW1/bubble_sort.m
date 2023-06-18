% bubble sort example
% test function 
a = [33, 103, 3, 726, 200, 984, 198, 764, 9]

% bublle sort a and b
a = bubbleSort(a)


%% bubble sort function in matlab
function a = bubbleSort(a)
    n = length(a);
    for i = 1:n-1
        for j = 1:n-i
            if a(j) > a(j+1)
                temp = a(j);
                a(j) = a(j+1);
                a(j+1) = temp;
            end
        end
    end
end