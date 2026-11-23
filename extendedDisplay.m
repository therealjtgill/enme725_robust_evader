classdef extendedDisplay
    %EXTENDEDDISPLAY Creates and adds to figures in extended displays
    %   Creates and remember fullscreen figures in extended displays. One
    %   of the figures provides a perspective transformation of the points
    %   being plotted to correct for projection position distortion.
    
    properties
        projMat
        hFigTV
        hFigPr
        hAxesTV
        hAxesPr
        hMasks
    end
    
    methods
        function obj = extendedDisplay(projMat,varargin)
            %EXTENDEDDISPLAY Construct an instance of this class
            %   Creates figure handles and stores projection matrix
            if nargin > 1
                obj.hFigTV = varargin{1};
                set(obj.hFigTV,'Units','normalized','Position',[1 1/3 2/3 2/3],'MenuBar','none','WindowState','fullscreen');
                obj.hAxesTV  = gca;
                set(obj.hAxesTV,'Units', 'normalized', 'Position', [0 0 1 1]);%,'XLim',[-2.6408 2.6408],'YLim',[-2.0828 2.0828]);
                hold(obj.hAxesTV,'on')
            else
                obj.hFigTV = figure('Units','normalized','Position',[1 0 1 1],'MenuBar','none','WindowState','fullscreen','Color','k');
                % Create TV figure axes
                obj.hAxesTV = axes(obj.hFigTV,'Units', 'normalized', 'Position', [0 0 1 1],'XLim',[-2.6408 2.6408],'YLim',[-2.0828 2.0828]);
                axis manual
                axis off
                hold(obj.hAxesTV,'on')
            end
            obj.hFigPr = figure('Units','normalized','Position',[-1 0 1 1],'MenuBar','none','Color','w','WindowState','fullscreen');
            % Create projector figure axes
            obj.hAxesPr = axes(obj.hFigPr,'Units', 'normalized','Position', [0 0 1 1],'XLim',[0 1],'YLim',[0 1]);
            axis manual
            axis off
            hold(obj.hAxesPr,'on')
            % Store projection matrix
            obj.projMat = projMat;
            % Create black masks on the projector
            obj.hMasks = obj.maskEdges();
        end
        
        function hMask = maskEdges(obj)
            % Create black masks on the projector
            % p = [1.6068   -1.6845   -2.5292   -2.5695    2.4966   2.5235;
            %   -1.7167   -1.7590   -0.3418    1.7699    1.8052   -0.2378]
            % P = obj.projectiveTransform(p);
            P = [     1         0         0    0.169    0.8408         1;
                 0.179    0.172    0.6783         1         1    0.6978].';
            P = [P ; 0 0 ; 0 1 ; 1 1 ; 1 0];
            % 1 to 2 to bottom left (7)
            % 1 to bottom left (7) to bottom right (10)
            % 3 to 4 to top left (8)
            % 5 to 6 to top right (9)
            F = [1 2 7; 1 7 10; 3 4 8 ; 5 6 9];
            hMask = patch(obj.hAxesPr,'Faces', F, 'Vertices', P,'FaceColor','k','EdgeColor','k');
        end
        
        function xc = projectiveTransform(obj,x)
            %projectiveTransform Transforms points from world to projector
            %   Uses a projective transformation to transform points in
            %   real world coordinates into distorted matlab figure
            %   coordinates so that the projected image is correct.
            %       - x: 2-by-m matrix of points in real world coordinate
            assert(size(x,1)==2,'Input must be a 2-by-m matrix')
            H   = obj.projMat ;
            num = H(1:2,1:2)*x + H(1:2,3);
            den = H( 3 ,1:2)*x + H( 3 ,3);
            xc  = num./den;
        end
        
        function [xc,yc] = projectiveTransform2(obj,x)
            %projectiveTransform Transforms points from world to projector
            %   Uses a projective transformation to transform points in
            %   real world coordinates into distorted matlab figure
            %   coordinates so that the projected image is correct.
            %       - x: 2-by-m matrix of points in real world coordinate
            assert(size(x,1)==2,'Input must be a 2-by-m matrix')
            H   = obj.projMat ;
            num = H(1:2,1:2)*x + H(1:2,3);
            den = H( 3 ,1:2)*x + H( 3 ,3);
            x  = num./den;
            xc = x(1,:);
            yc = x(2,:);
        end
        
        function [dXc,dYc] = projectiveTransformVelocities(obj,x,dx)
            %projectiveTransform Transforms points from world to projector
            %   Uses a projective transformation to transform points in
            %   real world coordinates into distorted matlab figure
            %   coordinates so that the projected image is correct.
            %       - x: 2-by-m matrix of points in real world coordinate
            assert(size(x,1)==2,'Input must be a 2-by-m matrix')
            H   = obj.projMat ;
            w = H( 3 ,1:2)*x + H( 3 ,3);
            wdot =  H( 3 ,1:2)*dx;
            X = (H(1:2,1:2)*x + H(1:2,3));
            num = H(1:2,1:2)*dx - wdot.*X;
            dX  = num./w;
            dXc = dX(1,:);
            dYc = dX(2,:);
        end
        function hPatch = patch(obj,X,Y,varargin)
            hPatch = gobjects(2,1);
            hPatch(1) = patch(obj.hAxesTV,'XData',X,'YData',Y,varargin{:});
            
            points = [X(:),Y(:)].';
            transformedPoints = projectiveTransform(obj,points);
            XData = transformedPoints(1,:);
            YData = transformedPoints(2,:);
            hPatch(2) = patch(obj.hAxesPr,'XData',XData,'YData',YData,varargin{:});
        end
        
         function hPatch = patchFV(obj,F,V,varargin)
            hPatch = gobjects(2,1);
            hPatch(1) = patch(obj.hAxesTV,'Faces',F,'Vertices',V,varargin{:});
            assert(size(V,2)==3,'Vertices are expected to be in a M-by-3 array')
            points = V(:,1:2).';
            transformedPoints = projectiveTransform(obj,points);
            XData = transformedPoints(1,:);
            YData = transformedPoints(2,:);
            hPatch(2) = patch(obj.hAxesPr,'Faces',F,'Vertices',[XData(:),YData(:),ones(size(XData(:)))],varargin{:});
        end
        
        function hFill = fill(obj,X,Y,varargin)
            hFill = gobjects(2,1);
            hFill(1) = fill(obj.hAxesTV,X,Y,varargin{:});
            
            points = [X(:),Y(:)].';
            transformedPoints = projectiveTransform(obj,points);
            XData = transformedPoints(1,:);
            YData = transformedPoints(2,:);
            hFill(2) = fill(obj.hAxesPr,XData,YData,varargin{:});
        end
        
        function hImplicit = fimplicit(obj, f, varargin)
            hImplicit = gobjects(2,1);
            hImplicit(1) = fimplicit(obj.hAxesTV, f, varargin{:});
            
            f_transformed = @(x,y) f([1,0]*projectiveTransform(obj,[x(:),y(:)].'),[0,1]*projectiveTransform(obj,[x(:),y(:)].'));
            hImplicit(2) = fimplicit(obj.hAxesPr, f_transformed, varargin{:});
        end
        
        function hLegend = legend(obj, varargin)
            hLegend = gobjects(2, 1);
        
            hLegend(1) = legend(obj.hAxesTV, varargin{:});
            
            hLegend(2) = legend(obj.hAxesPr, varargin{:});
        end
        
        function hScatter = scatter(obj,X,Y,varargin)
            hScatter = gobjects(2,1);
            hScatter(1) = scatter(obj.hAxesTV,X,Y,varargin{:});
            
            points = [X(:),Y(:)].';
            transformedPoints = projectiveTransform(obj,points);
            XData = transformedPoints(1,:);
            YData = transformedPoints(2,:);
            hScatter(2) = scatter(obj.hAxesPr,XData,YData,varargin{:});
        end
        
        function hPlot = plot(obj,X,Y,varargin)
            [m,n] = size(Y);
            if m==1 || n==1
                n = 1;
            end
            hPlot = gobjects(2,n);
            hPlot(1,:) = plot(obj.hAxesTV,X,Y,varargin{:});
            
            points = [X(:),Y(:)].';
            transformedPoints = projectiveTransform(obj,points);
            XData = transformedPoints(1,:);
            YData = transformedPoints(2,:);
            hPlot(2,:) = plot(obj.hAxesPr,XData,YData,varargin{:});
        end
                
        function hSurface = surface(obj,X,Y,C,varargin)
            hSurface = gobjects(2,1);
            hSurface(1) = surface(obj.hAxesTV,X,Y,zeros(size(X)),C,varargin{:},'EdgeColor','none','LineStyle','none');
%             hSurface(1) = image(obj.hAxesTV,X([1,end]),Y([1,end]),C,varargin{:});
            points = [X(:),Y(:)].';
            transformedPoints = projectiveTransform(obj,points);
            XData = reshape(transformedPoints(1,:),size(X));
            YData = reshape(transformedPoints(2,:),size(Y));
            %[XX,YY] = meshgrid(XData,YData);
%             hSurface(2) = surface(obj.hAxesPr,XData(1:4:end,1:4:end),YData(1:4:end,1:4:end),zeros(size(X(1:4:end,1:4:end))),C(1:4:end,1:4:end,:),'EdgeColor','none','LineStyle','none',varargin{:});
            hSurface(2) = surface(obj.hAxesPr,XData,YData,zeros(size(X)),C,'EdgeColor','none','LineStyle','none',varargin{:});
        end
                

        function hQuiver = quiver(obj,X,Y,U,V,varargin)
            hQuiver = gobjects(2,1);
            hQuiver(1) = quiver(obj.hAxesTV,X,Y,U,V,varargin{:});
            
            points = [X(:),Y(:)].';
            [XData,YData] = projectiveTransform2(obj,points);
            dPoints = [U(:),V(:)].';
            [UData,VData] = projectiveTransformVelocities(obj,points,dPoints);
            
            hQuiver(2) = quiver(obj.hAxesPr,XData,YData,UData,VData,varargin{:});
        end
        
        function hText = text(obj,X,Y,S,varargin)
            hText = gobjects(2,1);
            hText(1) = text(obj.hAxesTV,X,Y,S,varargin{:});
            points = [X(:),Y(:)].';
            transformedPoints = projectiveTransform(obj,points);
            XData = transformedPoints(1,:);
            YData = transformedPoints(2,:);
            hText(2) = text(obj.hAxesPr,XData,YData,S,varargin{:});
        end
        
        function set(obj,h,X,Y,varargin)
            assert(length(h)==2,'Two handles expected')
            set(h(1),'XData',X,'YData',Y,varargin{:})
            
            points = [X(:),Y(:)].';
            transformedPoints = projectiveTransform(obj,points);
            XData = transformedPoints(1,:);
            YData = transformedPoints(2,:);
            set(h(2),'XData',XData,'YData',YData,varargin{:});
        end
        
        function setQuiver(obj,h,X,Y,U,V,varargin)
            assert(length(h)==2,'Two handles expected')
            set(h(1),'XData',X,'YData',Y,'UData',U,'VData',V,varargin{:})
            
            points = [X(:),Y(:)].';
            [XData,YData] = projectiveTransform2(obj,points);
            dPoints = [U(:),V(:)].';
            [UData,VData] = projectiveTransformVelocities(obj,points,dPoints);
            set(h(2),'XData',XData,'YData',YData,'UData',UData,'VData',VData,varargin{:});
        end
        function setText(obj,h,X,Y,varargin)
            assert(length(h)==2,'Two handles expected')
            set(h(1),'Position',[X,Y],varargin{:})
            
            points = [X(:),Y(:)].';
            transformedPoints = projectiveTransform(obj,points);
            XData = transformedPoints(1,:);
            YData = transformedPoints(2,:);
            set(h(2),'Position',[XData,YData],varargin{:});
        end
        
        function setFV(obj,h,V,varargin)
            assert(length(h)==2,'Two handles expected')
            set(h(1),'Vertices',V,varargin{:})
            
            points = V(:,1:2).';
            transformedPoints = projectiveTransform(obj,points);
            XData = transformedPoints(1,:);
            YData = transformedPoints(2,:);
            set(h(2),'Vertices',[XData(:),YData(:),ones(size(XData(:)))],varargin{:});
        end
        
    end
end