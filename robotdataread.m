function [T] = robotdataread(filename,filedir)

%ROBOTDATAREAD  Opens and converts experiment datafiles 
%  This function reads in data from the specified 
%  fileid and converts it to ascii and saves it to
%  a structure. The following files are processed:
%  *loghead. dat : ascii triallist and config files 
%  *fpzero.dat   : forceplate calib files (binary to ascii)
%  *.dat         : trial data files (binary to ascii)
%     --called by : main_basic
%     --inputs    : filename, filedir
%     --outputs   : data structure, (T)
%     --calls     : robotconv.m
%  Created by Alaa Ahmed 19-Dec-2009
%  Last modified  15-Feb-2010

olddir=pwd;

if nargin==2
    cd(filedir)
end

filedat = dir(['*' filename '*.dat']);
[y,timesorted]=sort(cell2mat({filedat.datenum}'));
files_unsorted = {filedat.name}';
files=files_unsorted(timesorted);

istcl=cellfun('size',strfind(files,'tcl'),1);
isfpzero=cellfun('size',strfind(files,'fpzero'),1);
isheader=cellfun('size',strfind(files,'loghead'),1);
isdata=~istcl & ~isfpzero & ~isheader;

tclindex=find(istcl);
fpzeroindex=find(isfpzero & ~istcl) ;
headerindex=min(find(isheader));
dataindex=find(isdata);

datafiles=files(dataindex);
datafiles_char=char(datafiles);
underscoreindex=max(cell2mat(strfind(datafiles,'_')),[],2);
dotindex=cell2mat(strfind(datafiles,'.'));

ntrials=length(dataindex);
trialno=zeros(ntrials,1);

T.trials=ntrials;

%read data trials
for i=1:ntrials
            [fid,message] = fopen(datafiles{i});
            if fid > 0
                trialno(i)=str2num(datafiles_char(i,underscoreindex(i)+1:dotindex(i)-1));
                %if i ~= trialno(i)
                    %fprintf('Error: Trial Number mismatch for loop %d, trial %d \n',i,trialno(i));
                %else
                    %T.framedata(i)=robotconv(fid);
                    T.framedata(trialno(i))=robotconv(fid);
                    fprintf('Loop Number: %d; Trial %d/%d \n',trialno(i),trialno(i),ntrials);
                %end
                fclose(fid);
            else
                fprintf('Error: File for trial %d cannot open.\n',i);
            end
end

%read fpzero trials
for i=1:length(fpzeroindex)
            [fid,message] = fopen(files{fpzeroindex(i)});
            if fid > 0
                T.fpzerodata(i)=robotconv(fid);
                fprintf('File processed successfully:  %s.\n',files{fpzeroindex(i)});
                fclose(fid);
            else
                fprintf('Error: File %s cannot open.\n',files{fpzeroindex(i)});
            end
end

%read logheader
        fid = fopen(files{headerindex});
        i=1;
        while 1
            config{i} = fgets(fid);
            if strncmp(config{i},'trialnumber',10)==1, tableheader=config{i}; break,
            else
                [configvarname{i},configvarvalue{i}]=strtok(config{i});
                if ( ~isempty(configvarname{i}) && ~strncmp(configvarname{i},'/',1))
                    if isempty(str2num(configvarvalue{i}))
                        eval(['T.config.' configvarname{i}  '=configvarvalue{i};']);
                    else
                        eval(['T.config.' configvarname{i}  '=str2num(configvarvalue{i});']);
                    end
                end
                i= i+1; 
            end
        end
        
        rem=deblank(tableheader);
        formatstr=[repmat('%f',1,9) '%s%s' repmat('%f',1,sum(isspace(tableheader))-11)];
        TrialList_cell = textscan(fid,formatstr);
        fclose(fid);
        
        for i=1:sum(isspace(tableheader))
            [names{i},rem]=strtok(rem);
            if strfind(names{i},'(' )>1 names{i}=strrep(names{i},'(','_'); end
            if strfind(names{i},')' )>1 names{i}=strrep(names{i},')',''); end
            eval(['T.' names{i}  '=(TrialList_cell{i});'])
        end
        
        fprintf('File processed successfully:  %s.\n',files{headerindex});
        
cd(olddir);
    
    