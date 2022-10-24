function [x_hit,y_hit] = bresenham(x0,y0,x1,y1)
%BRESENHAM 
%https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm

if abs(y1 - y0) < abs(x1 - x0)
    if x0 > x1
        [x_hit,y_hit] = plotLineLow(x1,y1,x0,y0);
    else
        [x_hit,y_hit] = plotLineLow(x0,y0,x1,y1); 
    end
    
else
    if y0 > y1
        [x_hit,y_hit] = plotLineHigh(x1,y1,x0,y0);
    else
        [x_hit,y_hit] = plotLineHigh(x0,y0,x1,y1);
    end
end



    function [x_out, y_out] = plotLineLow(x0, y0, x1, y1)
        dx = x1 - x0;
        dy = y1 - y0;
        yi = 1;
        
        if dy < 0
            yi = -1;
            dy = -dy;
        end
        
        D = 2*dy - dx;
        y = y0;
        
        count = 1;
        for x=x0:1:x1
            x_out(count) = x;
            y_out(count) = y;
            
            if D > 0
                y = y + yi;
                D = D + 2*(dy - dx);
            else
                D = D + 2*dy;
            end
            count = count + 1;
        end 
    end

    function [x_out, y_out] = plotLineHigh(x0, y0, x1, y1)
        dx = x1 - x0;
        dy = y1 - y0;
        xi = 1;
        
        if dx < 0
            xi = -1;
            dx = -dx;
        end
        
        D = 2*dx - dy;
        x = x0;
        
        count = 1;
        for y=y0:1:y1
            x_out(count) = x;
            y_out(count) = y;
            
            if D > 0
                x = x + xi;
                D = D + 2*(dx - dy);
            else
                D = D + 2*dx;
            end
            count = count + 1;
        end 
    end

end

