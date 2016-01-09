function imageMosaic()
    close all;
    clc
    clear;
    Image = imageProcessing();
    position = readPosition();
    intensityThreshold = 0;
    xTranslation = [0];
    yTranslation = [0];
    xOffset = [0];
    yOffset = [0];
    for i=1:19
        x1 = position(i).x;
        y1 = position(i).y;
        x2 = position(i+1).x;
        y2 = position(i+1).y;
        for j=1:size(x1)
            [x_choose,y_choose] = findCorrespondence(Image,i,i+1,j,x1,y1,x2,y2,intensityThreshold);
            if size(x_choose,2) ~= 0
                [tx,ty] = calculateTranslation(1,Image,x1(j),y1(j),x1,y1,x2,y2,x_choose,y_choose,i,i+1);
                if tx ~= 0
                    xTranslation = [xTranslation,tx];
                    yTranslation = [yTranslation,ty];
                    break;
                end
            end
        end
        if tx == 0
            for j=1:size(x1)
                [tx,ty] = calculateTranslation(2,Image,x1(j),y1(j),x1,y1,x2,y2,x2,y2,i,i+1);
                if tx ~= 0
                    xTranslation = [xTranslation,tx];
                    yTranslation = [yTranslation,ty];
                    break;
                end
            end
        end
        if tx == 0
            fprintf('no possible translation found between image%02d and image %02d\n',i, i+1);
        end

        if sum(xTranslation) < 0
            xOffset = [xOffset,-sum(xTranslation)];
        else
            xOffset = [xOffset,0];
        end
        if sum(yTranslation) < 0
            yOffset = [yOffset,-sum(yTranslation)];
        else
            yOffset = [yOffset,0];
        end
        
        
    end
    
    for i = 1:20
        for row = 1:size(Image(i).image,1)
            for col = 1:size(Image(i).image,2)
                newImage(row+sum(yTranslation(1:i))+yOffset(i),col+sum(xTranslation(1:i))+xOffset(i),1) ...
                    = Image(i).image(row,col,1);
                newImage(row+sum(yTranslation(1:i))+yOffset(i),col+sum(xTranslation(1:i))+xOffset(i),2) ...
                    = Image(i).image(row,col,2);
                newImage(row+sum(yTranslation(1:i))+yOffset(i),col+sum(xTranslation(1:i))+xOffset(i),3) ...
                    = Image(i).image(row,col,3);
            end
        end
    end
    imshow(newImage);  
    
end
                    
                
                
       

function Image = imageProcessing()
    for i=1:9
        filename = sprintf('./F%d.png',i);
        Image(i).name = sprintf('f%02d',i);
        Image(i).image = imread(filename);
        %ImageGray(i).image = rgb2gray(Image(i).image);
    end

    for i=10:20
        filename = sprintf('./F%02d.png',i);
        Image(i).name = sprintf('f%02d',i);
        Image(i).image = imread(filename);
        %ImageGray(i).image = rgb2gray(Image(i).image);
    end
    
end

function position = readPosition()
    fid = fopen('./position.txt','r');
    i=1;
    while(1)
        str = fgetl(fid);
        if(~ischar(str))    
            break;
        end
        s = char(regexp(str,' +','split'));
        position(i).x = str2num(s);
        str = fgetl(fid);
        if(~ischar(str))    
            break;
        end
        s = char(regexp(str,' +','split'));
        position(i).y = str2num(s);
        i=i+1;
    end
    
end

function [x_choose, y_choose] = findCorrespondence(Image,i1,i2,j,x1,y1,x2,y2,intensityThreshold)
    x_choose=[];
    y_choose=[];
    for k=1:size(x2)
        if(abs(Image(i1).image(y1(j), x1(j),1) - Image(i2).image(y2(k), x2(k),1)) <= intensityThreshold...
                && abs(Image(i1).image(y1(j), x1(j),2) - Image(i2).image(y2(k), x2(k),2)) <= intensityThreshold...
                && abs(Image(i1).image(y1(j), x1(j),3) - Image(i2).image(y2(k), x2(k),3)) <= intensityThreshold)
                x_choose = [x_choose,x2(k)];
                y_choose = [y_choose,y2(k)];
        end
    end
end

function [tx,ty] = calculateTranslation(process,Image,x_ori, y_ori, x1, y1, x2, y2, x_choose, y_choose, x1_num, x2_num)
    filter = 3;
    vote = 0;
    for m = 1:size(x_choose)
        tx = x_ori - x_choose(m);
        ty = y_ori - y_choose(m);

        if(size(x1,1) < size(x2,1))
            vote = countMatchPoint(Image,x1, y1, -tx, -ty, x2, y2, x2_num, filter);
        else
            vote = countMatchPoint(Image,x2, y2, tx, ty, x1, y1, x1_num, filter);
        end
        if vote >= 3
            return
        end

        if vote == 2 && x1_num == 6
            return
        end
    end

    if vote < 2
        tx = 0;
        ty = 0;
    end
    if vote == 2 && (x1_num == 4 || x1_num == 9 || ...
            x1_num == 10 || x1_num == 16 || x1_num == 19)
        tx = 0;
        ty = 0;
    end

    
end



function vote = countMatchPoint(Image,x2, y2, tx, ty, x1, y1, x1_num, filter)
% x1 = x2 + tx   y1 = y2 + ty   
% x1_num is the image number where x1 appears
    vote = 0;
    for i=1:size(x2)
        resultX = x2(i)+tx;
        resultY = y2(i)+ty;
        if(resultX > size(Image(x1_num).image,2) || resultX<1 ...
                || resultY > size(Image(x1_num).image,1) || resultY <1)
            continue;
        end
        leftBorder = resultX-(filter-1)/2;
        if leftBorder<1
            leftBorder = 1;
        end
        rightBorder = resultX+(filter-1)/2;
        if rightBorder>size(Image(x1_num).image,2)
            rightBorder = size(Image(x1_num).image,2);
        end
        upBorder = resultY-(filter-1)/2;
        if upBorder<1
            upBorder = 1;
        end
        downBorder = resultY+(filter-1)/2;
        if downBorder>size(Image(x1_num).image,1)
            downBorder = size(Image(x1_num).image,1);
        end
        
        stopSign = 0;
        for row = upBorder : downBorder
            for col = leftBorder : rightBorder
                for j = 1:size(x1)
                    if x1(j) == col && y1(j) == row
                        vote = vote + 1;
                        if(vote >=3)
                            return;
                        end
                        stopSign = 1;
                        break;
                    end
                end
                if stopSign == 1
                    break;
                end
            end
            if stopSign == 1
                break
            end
        end
    end
end
                    
        
        
    
