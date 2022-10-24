%% Importer for .g2o files.
function [nodes, nodePairs, links, M] = importG2O(filename)
    hf = fopen(filename);
    numLinks = 1;
    while true
        line = fgetl(hf);
        if line == -1 | isempty(line)
            disp("Imported file")
            break
        end
        splitStr = split(line);

        if splitStr{1} == "EDGE_SE2"
            nodePairs(numLinks, 1:2) = [str2num(splitStr{2}) + 1, str2num(splitStr{3}) + 1];
            links(numLinks, 1:3) = [str2num(splitStr{4}), str2num(splitStr{5}), str2num(splitStr{6})];
            M(numLinks, 1:6) = [str2num(splitStr{7}), str2num(splitStr{8}), str2num(splitStr{9}),...
                                str2num(splitStr{10}), str2num(splitStr{11}), str2num(splitStr{12})];
            numLinks = numLinks + 1;
        elseif splitStr{1} == "VERTEX_SE2"
            nodes(str2num(splitStr{2})+1, 1) = str2num(splitStr{3});
            nodes(str2num(splitStr{2})+1, 2) = str2num(splitStr{4});
            nodes(str2num(splitStr{2})+1, 3) = str2num(splitStr{5});
        end
    end
end
    
    