%% Importer for .g2o files.
function [nodes, nodePairs, links, M] = importG2O_SE3(filename)
    hf = fopen(filename);
    numLinks = 1;
    while true
        line = fgetl(hf);
        if line == -1 | isempty(line)
            disp("Imported file")
            break
        end
        splitStr = split(line);

        if splitStr{1} == "EDGE_SE3:QUAT"
            nodePairs(numLinks, 1:2) = [str2num(splitStr{2}) + 1, str2num(splitStr{3}) + 1];
            
            links(numLinks, 1:3) = [str2double(splitStr{4}), str2double(splitStr{5}), str2double(splitStr{6})];
            xyz = quat2eul([str2double(splitStr{10}), str2double(splitStr{7}), str2double(splitStr{8}), str2double(splitStr{9})] , "XYZ");
            links(numLinks, 1:3) = xyz;
            M(numLinks, 1:21) = [str2double(splitStr{7}), str2double(splitStr{8}), str2double(splitStr{9}),...
                                str2double(splitStr{10}), str2double(splitStr{11}), str2double(splitStr{12})...
                                str2double(splitStr{13}), str2double(splitStr{14}), str2double(splitStr{15})...
                                str2double(splitStr{16}), str2double(splitStr{16}), str2double(splitStr{18})...
                                str2double(splitStr{19}), str2double(splitStr{20}), str2double(splitStr{21})...
                                str2double(splitStr{22}), str2double(splitStr{23}), str2double(splitStr{24})...
                                str2double(splitStr{25}), str2double(splitStr{26}), str2double(splitStr{27})];
            numLinks = numLinks + 1;
            
        elseif splitStr{1} == "VERTEX_SE3:QUAT"
            % x y z qx qy qz w
            nodes(str2num(splitStr{2})+1, 1) = str2double(splitStr{3});
            nodes(str2num(splitStr{2})+1, 2) = str2double(splitStr{4});
            nodes(str2num(splitStr{2})+1, 3) = str2double(splitStr{5});
            
            xyz = quat2eul([str2double(splitStr{9}), str2double(splitStr{6}), str2double(splitStr{7}), str2double(splitStr{8})] , "XYZ");
            nodes(str2num(splitStr{2})+1, 4) = xyz(1);
            nodes(str2num(splitStr{2})+1, 5) = xyz(2);
            nodes(str2num(splitStr{2})+1, 6) = xyz(3);
        end
    end
end
    
    