function png_bitmap_gen(filename, spritenum, spritesize, invertcol)
% change this to the path name of the sprite folder
folder_path = 'C:\Users\[username]\OneDrive\Documents\MATLAB\Sprites\';
% pretty self explainatory lol
default_sprite_name = 'sprite';


% assumes sprite is square, and black/white
% export sprite as nx1 (rows,columns), where n is the number of frames
% filename should be the name of the file without the file type
% i.e. 'claw-image' for a file called 'claw-image.png
% file extension is assumed to tbe png
% spritenum is the number of sprites in animation
% sprite size is width of the sprite. Sprite canvas size is assumed to be a square multiple of 8
% setting invertcol to true swaps white/black (probably :D)


    switch (nargin)
        case 3
            invertcol = true;
        case 2
            spritesize = 32;
            invertcol = true;
        case 1
            spritesize = 32;
            invertcol = true;
            spritenum = 1;
        case 0
            spritesize = 32;
            invertcol = true;
            spritenum = 1;
            filename = default_sprite_name;
    end
    currentframe = 1;
    pngcell = cell(spritenum)
    for i = 1:spritenum

        pngcell{i} = bit_gen(filename, spritesize, invertcol, currentframe, spritenum, folder_path);
        currentframe = currentframe + 1;
    end
    create_txt(pngcell);
end

function pngcell = bit_gen(filename, spritesize, invertcol, currentframe, spritenum, file_path)

    [X,map,alpha] = imread(cat(2, file_path, filename,'.png'));
    if alpha == zeros(spritesize*spritenum,spritesize)
        pnginfo = X;
    elseif alpha == ones(spritesize*spritenum,spritesize)*255
        pnginfo = X;
    else
        pnginfo = alpha;
        invertcol = ~invertcol;
    end
    %%assuming black and white sprites. Colored sprites don't work
    pnginfo = pnginfo(:,:,1); %guessing 1 = r, 2 = g, 3 = b?? Not sure. We can pick any
    pnginfo = pnginfo/255;
    
    %swaps zeros and ones if inverted is desired
    if invertcol == true
        pnginfo = uint8(~pnginfo);
    end
        
    %%deletes all frames but the current one
        pnginfo = pnginfo(1+(currentframe-1)*spritesize:currentframe*spritesize,:);
    %re-enters png bit info into cells, as strings
    pngcell = cell(spritesize, spritesize/8);
    for i = 1:spritesize
        for j = 1:spritesize/8
            pngcell{i,j} = num2str(pnginfo(i + spritesize * ((j-1)*8 : (j*8 - 1))));
            %removes spaces in each cell
            pngcell{i,j} = pngcell{i,j}(find(~isspace(pngcell{i,j})));
            %adds 0b to the front of each cell
            pngcell{i,j} = cat(2, '0b', pngcell{i,j});
        end
        %% adds a comma to every cell but the last
        if i ~= spritesize 
            pngcell{i,(spritesize/8)} = cat(2, pngcell{i,(spritesize/8)}, ',');
        end
        %concatenates columns into first column, with spaces in between
        pngcell{i,1} = strjoin(pngcell(i,:));
    end
    %deletes every column but the first
    pngcell(:,2:end) = [];
    
    %adds other mumbo jumbo so that arduino ide can process the sprite
    pngcell(2:end+1,1) = pngcell(:,1);
    pngcell{1} =  cat(2, 'static const unsigned char PROGMEM ', filename, '_', num2str(currentframe), '[] = {');
    pngcell{end+1} = '};';
end

function create_txt(pngcell)
    %writes into a txt file
    
    writelines([pngcell{1:end}],'map.txt');
    type map.txt;
end
