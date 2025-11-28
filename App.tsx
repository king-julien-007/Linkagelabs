
import React, { useState, useRef, useEffect, useCallback, useMemo } from 'react';
import Toolbar from './components/Toolbar';
import AICreator from './components/AICreator';
import NodeInspector from './components/NodeInspector';
import Sidebar from './components/Sidebar';
import { ToolType, Node, Link, MechanismState, GeneratedMechanism } from './types';
import { PresetDefinition } from './data/presets';
import { nanoid } from 'nanoid';

const SNAP_DISTANCE = 15;
const BASE_MOTOR_SPEED = 0.05;

const App: React.FC = () => {
  // --- State ---
  const [nodes, setNodes] = useState<Node[]>([]);
  const [links, setLinks] = useState<Link[]>([]);
  
  const [activeTool, setActiveTool] = useState<ToolType>(ToolType.SELECT);
  const [isPlaying, setIsPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [showTraces, setShowTraces] = useState(false);
  const [isAIModalOpen, setIsAIModalOpen] = useState(false);
  const [isSidebarOpen, setIsSidebarOpen] = useState(false);
  
  // Viewport State (Pan & Zoom)
  const [viewport, setViewport] = useState({ x: 0, y: 0, scale: 1 });
  const [isPanning, setIsPanning] = useState(false);
  const lastMousePos = useRef<{x: number, y: number} | null>(null);
  
  // Drawing Target Path State
  const [targetPath, setTargetPath] = useState<{x: number, y: number}[]>([]);
  const [isDrawingPath, setIsDrawingPath] = useState(false);

  // Interaction State
  const [selectedNodeId, setSelectedNodeId] = useState<string | null>(null);
  const [hoveredNodeId, setHoveredNodeId] = useState<string | null>(null);
  const [draggedNodeId, setDraggedNodeId] = useState<string | null>(null);
  const [linkStartNodeId, setLinkStartNodeId] = useState<string | null>(null);
  const [tempLinkEnd, setTempLinkEnd] = useState<{x: number, y: number} | null>(null);

  // Canvas Refs
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const animationFrameRef = useRef<number>();
  const tracesRef = useRef<Record<string, {x: number, y: number}[]>>({});

  // --- Coordinate Systems ---
  
  // Convert Screen pixel (clientX) to World coordinate (Node position)
  const screenToWorld = useCallback((sx: number, sy: number) => {
    return {
      x: (sx - viewport.x) / viewport.scale,
      y: (sy - viewport.y) / viewport.scale
    };
  }, [viewport]);

  // Convert World coordinate to Screen pixel
  const worldToScreen = useCallback((wx: number, wy: number) => {
    return {
      x: wx * viewport.scale + viewport.x,
      y: wy * viewport.scale + viewport.y
    };
  }, [viewport]);

  // Helper to find node under cursor (uses World Coordinates)
  const getNodeAt = (worldX: number, worldY: number) => {
    // Snap distance needs to be visually consistent, so we divide by scale? 
    // Actually, simple euclidean distance in world space works best.
    // However, if we zoom out, clicking becomes hard if we require strict 15px world distance.
    // Let's adjust snap distance based on scale to keep it roughly "screen pixel" consistent.
    const effectiveSnap = SNAP_DISTANCE / viewport.scale;
    return nodes.find(n => Math.hypot(n.x - worldX, n.y - worldY) < effectiveSnap);
  };

  // Get the actual selected node object
  const selectedNode = nodes.find(n => n.id === selectedNodeId);

  // --- Path Geometry Helpers ---
  const pathMetrics = useMemo(() => {
    if (targetPath.length < 2) return { totalLength: 0, segments: [] };
    
    let totalLength = 0;
    const segments = [];
    
    for (let i = 0; i < targetPath.length - 1; i++) {
      const p1 = targetPath[i];
      const p2 = targetPath[i+1];
      const len = Math.hypot(p2.x - p1.x, p2.y - p1.y);
      segments.push({
        start: p1,
        end: p2,
        length: len,
        accumulatedStart: totalLength
      });
      totalLength += len;
    }
    return { totalLength, segments };
  }, [targetPath]);

  const getPointOnPath = useCallback((distance: number) => {
    const { totalLength, segments } = pathMetrics;
    if (totalLength === 0 || segments.length === 0) return null;

    // Normalize distance to loop
    let d = distance % totalLength;
    if (d < 0) d += totalLength;

    // Find segment
    const segment = segments.find(s => d >= s.accumulatedStart && d <= s.accumulatedStart + s.length) 
                    || segments[segments.length - 1];

    const segmentOffset = d - segment.accumulatedStart;
    const t = segmentOffset / segment.length; // 0 to 1

    return {
      x: segment.start.x + (segment.end.x - segment.start.x) * t,
      y: segment.start.y + (segment.end.y - segment.start.y) * t
    };
  }, [pathMetrics]);

  // --- Gruebler's Criterion Calculation ---
  const mobilityInfo = useMemo(() => {
    // 1. Filter out links that are purely ground (fixed-to-fixed)
    const activeLinks = links.filter(link => {
      const source = nodes.find(n => n.id === link.sourceId);
      const target = nodes.find(n => n.id === link.targetId);
      if (!source || !target) return false;
      return !(source.isFixed && target.isFixed);
    });

    const N = activeLinks.length + 1;

    // 2. Calculate Joints (J)
    let J = 0;
    const nodeConnectionCounts = new Map<string, number>();
    
    activeLinks.forEach(link => {
      nodeConnectionCounts.set(link.sourceId, (nodeConnectionCounts.get(link.sourceId) || 0) + 1);
      nodeConnectionCounts.set(link.targetId, (nodeConnectionCounts.get(link.targetId) || 0) + 1);
    });

    nodes.forEach(node => {
      const m = nodeConnectionCounts.get(node.id) || 0;
      if (m > 0) {
        if (node.isFixed) {
          J += m;
        } else {
          J += m - 1;
        }
      }
    });

    // M = 3(N - 1) - 2J
    const mobility = 3 * (N - 1) - 2 * J;

    let status = "Structure";
    let color = "text-gray-600";

    if (activeLinks.length === 0) {
      status = "Empty";
    } else if (mobility < 0) {
      status = "Overconstrained";
      color = "text-red-600";
    } else if (mobility === 0) {
      status = "Rigid Structure";
      color = "text-amber-600";
    } else if (mobility === 1) {
      status = "Mechanism (1 DOF)";
      color = "text-emerald-600";
    } else {
      status = "Underconstrained";
      color = "text-blue-600";
    }

    return { mobility, status, color };
  }, [nodes, links]);

  // --- Actions ---

  const handleWheel = (e: React.WheelEvent) => {
    // e.preventDefault(); // Note: React synth events might not allow preventDefault on passive listeners
    const rect = canvasRef.current!.getBoundingClientRect();
    const screenX = e.clientX - rect.left;
    const screenY = e.clientY - rect.top;

    // Get point under mouse before zoom
    const worldPos = screenToWorld(screenX, screenY);

    const zoomSensitivity = 0.001;
    const delta = -e.deltaY * zoomSensitivity;
    
    // Calculate new scale with clamping
    const newScale = Math.min(Math.max(0.1, viewport.scale + delta), 5);
    
    // Calculate new offset so the world point remains under the mouse
    // screenX = worldX * newScale + newViewportX
    // newViewportX = screenX - worldX * newScale
    const newX = screenX - worldPos.x * newScale;
    const newY = screenY - worldPos.y * newScale;

    setViewport({
      x: newX,
      y: newY,
      scale: newScale
    });
  };

  const handleCanvasMouseDown = (e: React.MouseEvent) => {
    const rect = canvasRef.current!.getBoundingClientRect();
    const screenX = e.clientX - rect.left;
    const screenY = e.clientY - rect.top;
    
    // Middle click or Spacebar held -> PAN
    if (e.button === 1 || e.getModifierState('Space')) {
      setIsPanning(true);
      lastMousePos.current = { x: screenX, y: screenY };
      return;
    }

    const { x: worldX, y: worldY } = screenToWorld(screenX, screenY);
    
    if (activeTool === ToolType.DRAW_PATH) {
      setIsDrawingPath(true);
      setTargetPath([{x: worldX, y: worldY}]); // Start new path
      return;
    }

    const clickedNode = getNodeAt(worldX, worldY);

    if (activeTool === ToolType.SELECT) {
      if (clickedNode) {
        setDraggedNodeId(clickedNode.id);
        setSelectedNodeId(clickedNode.id); // Select node
      } else {
        setSelectedNodeId(null); // Deselect if clicking empty space
        // If clicking empty space in select mode, we can also allow panning
        setIsPanning(true);
        lastMousePos.current = { x: screenX, y: screenY };
      }
    } else if (activeTool === ToolType.JOINT) {
      const newNode: Node = {
        id: nanoid(),
        x: worldX,
        y: worldY,
        isFixed: false,
        isMotor: false
      };
      setNodes(prev => [...prev, newNode]);
    } else if (activeTool === ToolType.GROUND) {
      if (clickedNode) {
        setNodes(prev => prev.map(n => n.id === clickedNode.id ? { ...n, isFixed: true } : n));
      } else {
        const newNode: Node = {
          id: nanoid(),
          x: worldX,
          y: worldY,
          isFixed: true,
          isMotor: false
        };
        setNodes(prev => [...prev, newNode]);
      }
    } else if (activeTool === ToolType.LINK) {
      if (clickedNode) {
        setLinkStartNodeId(clickedNode.id);
        setTempLinkEnd({ x: worldX, y: worldY });
      }
    } else if (activeTool === ToolType.MOTOR) {
      if (clickedNode) {
        const isNowMotor = !clickedNode.isMotor;
        
        let pivotId = undefined;
        let radius = undefined;
        let angle = undefined;

        if (isNowMotor) {
          // Find a fixed neighbor to be the pivot
          const connectedLink = links.find(l => l.sourceId === clickedNode.id || l.targetId === clickedNode.id);
          if (connectedLink) {
            const neighborId = connectedLink.sourceId === clickedNode.id ? connectedLink.targetId : connectedLink.sourceId;
            const neighbor = nodes.find(n => n.id === neighborId);
            if (neighbor && neighbor.isFixed) {
              pivotId = neighbor.id;
              radius = connectedLink.length;
              angle = Math.atan2(clickedNode.y - neighbor.y, clickedNode.x - neighbor.x);
            }
          }
        }

        setNodes(prev => prev.map(n => n.id === clickedNode.id ? { 
          ...n, 
          isMotor: isNowMotor,
          pivotId,
          radius,
          angle,
          isFixed: false // Motor cannot be fixed
        } : n));
      }
    } else if (activeTool === ToolType.DELETE) {
      if (clickedNode) {
        setNodes(prev => prev.filter(n => n.id !== clickedNode.id));
        setLinks(prev => prev.filter(l => l.sourceId !== clickedNode.id && l.targetId !== clickedNode.id));
        if (tracesRef.current[clickedNode.id]) {
          delete tracesRef.current[clickedNode.id];
        }
        if (selectedNodeId === clickedNode.id) {
          setSelectedNodeId(null);
        }
      }
    }
  };

  const handleCanvasMouseMove = (e: React.MouseEvent) => {
    const rect = canvasRef.current!.getBoundingClientRect();
    const screenX = e.clientX - rect.left;
    const screenY = e.clientY - rect.top;
    
    // Handle Panning
    if (isPanning && lastMousePos.current) {
      const dx = screenX - lastMousePos.current.x;
      const dy = screenY - lastMousePos.current.y;
      setViewport(prev => ({
        ...prev,
        x: prev.x + dx,
        y: prev.y + dy
      }));
      lastMousePos.current = { x: screenX, y: screenY };
      return;
    }

    const { x: worldX, y: worldY } = screenToWorld(screenX, screenY);

    if (activeTool === ToolType.DRAW_PATH && isDrawingPath) {
      const lastPoint = targetPath[targetPath.length - 1];
      if (!lastPoint || Math.hypot(lastPoint.x - worldX, lastPoint.y - worldY) > 5) {
        setTargetPath(prev => [...prev, {x: worldX, y: worldY}]);
      }
      return;
    }

    const hovered = getNodeAt(worldX, worldY);
    setHoveredNodeId(hovered ? hovered.id : null);

    if (draggedNodeId) {
      setNodes(prev => prev.map(n => {
        if (n.id === draggedNodeId) {
          // If dragging a motor, update its radius/angle relative to pivot
          if (n.isMotor && n.pivotId && !n.isPathFollower) {
             const pivot = prev.find(p => p.id === n.pivotId);
             if (pivot) {
                const dx = worldX - pivot.x;
                const dy = worldY - pivot.y;
                return { ...n, x: worldX, y: worldY, angle: Math.atan2(dy, dx), radius: Math.sqrt(dx*dx + dy*dy) };
             }
          }
          return { ...n, x: worldX, y: worldY };
        }
        return n;
      }));
      
      if (!isPlaying) {
         setLinks(prev => prev.map(l => {
           const start = nodes.find(n => n.id === l.sourceId);
           const end = nodes.find(n => n.id === l.targetId);
           if (l.sourceId === draggedNodeId && end) {
             return { ...l, length: Math.hypot(worldX - end.x, worldY - end.y) };
           }
           if (l.targetId === draggedNodeId && start) {
             return { ...l, length: Math.hypot(start.x - worldX, start.y - worldY) };
           }
           return l;
         }));
         tracesRef.current = {};
      }
    }

    if (linkStartNodeId) {
      setTempLinkEnd({ x: worldX, y: worldY });
    }
  };

  const handleCanvasMouseUp = (e: React.MouseEvent) => {
    if (isPanning) {
      setIsPanning(false);
      lastMousePos.current = null;
      return;
    }

    if (activeTool === ToolType.DRAW_PATH) {
      setIsDrawingPath(false);
      return;
    }

    const rect = canvasRef.current!.getBoundingClientRect();
    const screenX = e.clientX - rect.left;
    const screenY = e.clientY - rect.top;
    const { x: worldX, y: worldY } = screenToWorld(screenX, screenY);
    
    if (draggedNodeId) {
      setDraggedNodeId(null);
    }

    if (linkStartNodeId) {
      const endNode = getNodeAt(worldX, worldY);
      if (endNode && endNode.id !== linkStartNodeId) {
        const startNode = nodes.find(n => n.id === linkStartNodeId)!;
        const length = Math.hypot(endNode.x - startNode.x, endNode.y - startNode.y);
        
        const exists = links.some(l => 
          (l.sourceId === linkStartNodeId && l.targetId === endNode.id) ||
          (l.sourceId === endNode.id && l.targetId === linkStartNodeId)
        );

        if (!exists) {
          setLinks(prev => [...prev, {
            id: nanoid(),
            sourceId: linkStartNodeId,
            targetId: endNode.id,
            length
          }]);
        }
      }
      setLinkStartNodeId(null);
      setTempLinkEnd(null);
    }
  };

  // --- Drag & Drop Handlers for Presets ---
  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = 'copy';
  };

  const handleDrop = (e: React.DragEvent) => {
    e.preventDefault();
    const data = e.dataTransfer.getData('application/json');
    if (!data) return;

    try {
      const preset: PresetDefinition = JSON.parse(data);
      const rect = canvasRef.current!.getBoundingClientRect();
      const dropScreenX = e.clientX - rect.left;
      const dropScreenY = e.clientY - rect.top;
      
      // Convert drop point to world coordinates
      const { x: dropX, y: dropY } = screenToWorld(dropScreenX, dropScreenY);

      const idMap: Record<string, string> = {};
      
      const newNodes: Node[] = preset.nodes.map(pn => {
        const newId = nanoid();
        idMap[pn.id] = newId;
        const { xOffset, yOffset, id: _oldId, ...rest } = pn;
        return {
          ...rest,
          id: newId,
          x: dropX + xOffset,
          y: dropY + yOffset,
        };
      });

      // Update pivotIds in the new nodes now that we have the full idMap
      const finalNodes = newNodes.map(n => {
        if (n.pivotId && idMap[n.pivotId]) {
          return { ...n, pivotId: idMap[n.pivotId] };
        }
        return n;
      });

      const newLinks: Link[] = preset.links.map(pl => {
        const sourceId = idMap[pl.source];
        const targetId = idMap[pl.target];
        const sourceNode = finalNodes.find(n => n.id === sourceId);
        const targetNode = finalNodes.find(n => n.id === targetId);
        
        if (sourceNode && targetNode) {
          return {
            id: nanoid(),
            sourceId,
            targetId,
            length: Math.hypot(targetNode.x - sourceNode.x, targetNode.y - sourceNode.y)
          };
        }
        return null;
      }).filter(Boolean) as Link[];

      setNodes(prev => [...prev, ...finalNodes]);
      setLinks(prev => [...prev, ...newLinks]);

    } catch (err) {
      console.error("Failed to parse dropped preset", err);
    }
  };

  // --- Inspector Updates ---
  const handleNodeUpdate = (updates: Partial<Node>) => {
    if (!selectedNodeId) return;

    setNodes(prev => prev.map(n => {
      if (n.id !== selectedNodeId) return n;

      const updatedNode = { ...n, ...updates };

      // Handle logic when toggling isMotor ON
      if (updates.isMotor === true && !n.isMotor) {
        // Try to find a pivot automatically
        const connectedLink = links.find(l => l.sourceId === n.id || l.targetId === n.id);
        if (connectedLink) {
          const neighborId = connectedLink.sourceId === n.id ? connectedLink.targetId : connectedLink.sourceId;
          const neighbor = prev.find(p => p.id === neighborId);
          if (neighbor && neighbor.isFixed) {
             updatedNode.pivotId = neighbor.id;
             updatedNode.radius = connectedLink.length;
             updatedNode.angle = Math.atan2(n.y - neighbor.y, n.x - neighbor.x);
          }
        }
      }

      // If angle changed manually, update X/Y
      if (updates.angle !== undefined && n.pivotId && !n.isPathFollower) {
        const pivot = prev.find(p => p.id === n.pivotId);
        if (pivot) {
           updatedNode.x = pivot.x + Math.cos(updates.angle) * (n.radius || 50);
           updatedNode.y = pivot.y + Math.sin(updates.angle) * (n.radius || 50);
        }
      }

      // If X or Y changed manually for a motor, update angle/radius
      if ((updates.x !== undefined || updates.y !== undefined) && n.isMotor && n.pivotId && !n.isPathFollower) {
         const pivot = prev.find(p => p.id === n.pivotId);
         if (pivot) {
            const nx = updates.x ?? n.x;
            const ny = updates.y ?? n.y;
            updatedNode.angle = Math.atan2(ny - pivot.y, nx - pivot.x);
            updatedNode.radius = Math.hypot(nx - pivot.x, ny - pivot.y);
         }
      }

      return updatedNode;
    }));

    // Update links if position changed
    if (updates.x !== undefined || updates.y !== undefined) {
      setLinks(prev => prev.map(l => {
        if (l.sourceId === selectedNodeId || l.targetId === selectedNodeId) {
          const start = nodes.find(n => n.id === l.sourceId);
          const end = nodes.find(n => n.id === l.targetId);
          // Use updated coords for current node
          const sX = l.sourceId === selectedNodeId ? (updates.x ?? 0) : start?.x ?? 0;
          const sY = l.sourceId === selectedNodeId ? (updates.y ?? 0) : start?.y ?? 0;
          const eX = l.targetId === selectedNodeId ? (updates.x ?? 0) : end?.x ?? 0;
          const eY = l.targetId === selectedNodeId ? (updates.y ?? 0) : end?.y ?? 0;
          return { ...l, length: Math.hypot(eX - sX, eY - sY) };
        }
        return l;
      }));
    }
  };


  // --- Physics Solver ---

  const solveConstraints = useCallback(() => {
    const newNodes = nodes.map(node => {
      if (node.isMotor && isPlaying) {
        
        // --- PATH FOLLOWER MODE ---
        if (node.isPathFollower && targetPath.length > 1) {
          const motorSpeedMultiplier = node.speed ?? 1;
          const delta = BASE_MOTOR_SPEED * speed * motorSpeedMultiplier;
          // Use 'angle' as the ticker/accumulator for distance traveled along path
          const currentTicker = node.angle || 0;
          const newTicker = currentTicker + delta;
          
          // Map ticker to pixels (e.g., 1 radian ~= 100 pixels for reasonable speed)
          const distance = newTicker * 100; 
          
          const point = getPointOnPath(distance);
          
          if (point) {
            return {
              ...node,
              angle: newTicker,
              x: point.x,
              y: point.y
            };
          }
        } 
        
        // --- STANDARD ROTARY MOTOR ---
        else if (node.pivotId) {
          const pivot = nodes.find(n => n.id === node.pivotId);
          if (pivot) {
            const motorSpeedMultiplier = node.speed ?? 1;
            const newAngle = (node.angle || 0) + (BASE_MOTOR_SPEED * speed * motorSpeedMultiplier);
            const r = node.radius || 50;
            return {
              ...node,
              angle: newAngle,
              x: pivot.x + Math.cos(newAngle) * r,
              y: pivot.y + Math.sin(newAngle) * r
            };
          }
        }
      }
      return node;
    });

    const iterations = 10;
    const currentNodes = [...newNodes];

    for (let i = 0; i < iterations; i++) {
      links.forEach(link => {
        const n1Index = currentNodes.findIndex(n => n.id === link.sourceId);
        const n2Index = currentNodes.findIndex(n => n.id === link.targetId);
        if (n1Index === -1 || n2Index === -1) return;

        const n1 = currentNodes[n1Index];
        const n2 = currentNodes[n2Index];

        const dx = n2.x - n1.x;
        const dy = n2.y - n1.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        
        if (dist === 0) return;

        const diff = (dist - link.length) / dist;
        const moveX = dx * diff * 0.5;
        const moveY = dy * diff * 0.5;

        const n1IsLocked = n1.isFixed || n1.isMotor || n1.id === draggedNodeId;
        const n2IsLocked = n2.isFixed || n2.isMotor || n2.id === draggedNodeId;

        if (!n1IsLocked) {
          currentNodes[n1Index].x += moveX;
          currentNodes[n1Index].y += moveY;
        }
        if (!n2IsLocked) {
          currentNodes[n2Index].x -= moveX;
          currentNodes[n2Index].y -= moveY;
        }
        
        if (n1IsLocked && !n2IsLocked) {
          currentNodes[n2Index].x -= moveX;
          currentNodes[n2Index].y -= moveY;
        }
        if (!n1IsLocked && n2IsLocked) {
          currentNodes[n1Index].x += moveX;
          currentNodes[n1Index].y += moveY;
        }
      });
    }

    setNodes(currentNodes);
  }, [nodes, links, isPlaying, speed, draggedNodeId, targetPath, getPointOnPath]);


  // --- Render Loop ---
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    
    // Initial Setup
    ctx.resetTransform(); // Clear any previous transform
    ctx.scale(dpr, dpr); 

    const render = () => {
      ctx.save(); // Save the identity state
      ctx.clearRect(0, 0, rect.width, rect.height);

      // --- Apply Viewport Transform ---
      ctx.translate(viewport.x, viewport.y);
      ctx.scale(viewport.scale, viewport.scale);

      // --- Draw Infinite Grid ---
      const gridSpacing = 50;
      // Calculate visible bounds in world space
      const visibleMinX = (0 - viewport.x) / viewport.scale;
      const visibleMaxX = (rect.width - viewport.x) / viewport.scale;
      const visibleMinY = (0 - viewport.y) / viewport.scale;
      const visibleMaxY = (rect.height - viewport.y) / viewport.scale;

      const startX = Math.floor(visibleMinX / gridSpacing) * gridSpacing;
      const startY = Math.floor(visibleMinY / gridSpacing) * gridSpacing;

      ctx.strokeStyle = '#e5e7eb';
      ctx.lineWidth = 1 / viewport.scale; // Maintain hairline width regardless of zoom
      ctx.beginPath();

      for (let x = startX; x <= visibleMaxX; x += gridSpacing) {
        ctx.moveTo(x, visibleMinY);
        ctx.lineTo(x, visibleMaxY);
      }
      for (let y = startY; y <= visibleMaxY; y += gridSpacing) {
        ctx.moveTo(visibleMinX, y);
        ctx.lineTo(visibleMaxX, y);
      }
      ctx.stroke();

      // Draw Target Path
      if (targetPath.length > 1) {
        ctx.beginPath();
        ctx.moveTo(targetPath[0].x, targetPath[0].y);
        for(let i = 1; i < targetPath.length; i++) {
          ctx.lineTo(targetPath[i].x, targetPath[i].y);
        }
        ctx.strokeStyle = 'rgba(34, 197, 94, 0.5)'; // Green
        ctx.lineWidth = 4;
        ctx.setLineDash([10, 10]);
        ctx.stroke();
        ctx.setLineDash([]);
        
        ctx.fillStyle = '#16a34a';
        ctx.beginPath();
        ctx.arc(targetPath[0].x, targetPath[0].y, 4, 0, Math.PI*2);
        ctx.arc(targetPath[targetPath.length-1].x, targetPath[targetPath.length-1].y, 4, 0, Math.PI*2);
        ctx.fill();
      }

      // Draw Traces
      if (showTraces) {
        Object.entries(tracesRef.current).forEach(([id, points]) => {
          const tracePoints = points as {x: number, y: number}[];
          if (tracePoints.length < 2) return;
          ctx.beginPath();
          ctx.moveTo(tracePoints[0].x, tracePoints[0].y);
          for (let i = 1; i < tracePoints.length; i++) {
            ctx.lineTo(tracePoints[i].x, tracePoints[i].y);
          }
          ctx.strokeStyle = 'rgba(147, 51, 234, 0.4)'; // Purple
          ctx.lineWidth = 3;
          ctx.stroke();
        });
      }

      // Links (Render as 2D mechanical parts)
      links.forEach(link => {
        const start = nodes.find(n => n.id === link.sourceId);
        const end = nodes.find(n => n.id === link.targetId);
        if (start && end) {
          // Calculate geometry
          const dx = end.x - start.x;
          const dy = end.y - start.y;
          const length = Math.hypot(dx, dy);
          const angle = Math.atan2(dy, dx);
          
          const linkWidth = 20;

          ctx.save();
          ctx.translate(start.x, start.y);
          ctx.rotate(angle);

          // Draw Link Body (Capsule)
          ctx.beginPath();
          // Left Cap (Semicircle)
          ctx.arc(0, 0, linkWidth / 2, Math.PI / 2, -Math.PI / 2);
          // Top Edge
          ctx.lineTo(length, -linkWidth / 2);
          // Right Cap (Semicircle)
          ctx.arc(length, 0, linkWidth / 2, -Math.PI / 2, Math.PI / 2);
          // Bottom Edge
          ctx.lineTo(0, linkWidth / 2);
          ctx.closePath();

          // Style
          ctx.fillStyle = '#cbd5e1'; // Metallic Grey Fill
          ctx.fill();
          ctx.lineWidth = 2;
          ctx.strokeStyle = '#475569'; // Dark Slate Border
          ctx.stroke();

          // Center Detail (Groove) - Only if link is long enough
          if (length > linkWidth) {
             ctx.beginPath();
             ctx.moveTo(linkWidth / 2, 0);
             ctx.lineTo(length - linkWidth / 2, 0);
             ctx.lineWidth = 2;
             ctx.strokeStyle = 'rgba(0, 0, 0, 0.1)';
             ctx.stroke();
          }

          // Pin Holes (Visual)
          ctx.fillStyle = '#334155';
          ctx.beginPath();
          ctx.arc(0, 0, 4, 0, Math.PI * 2);
          ctx.arc(length, 0, 4, 0, Math.PI * 2);
          ctx.fill();

          ctx.restore();
        }
      });

      // Temp Link
      if (linkStartNodeId && tempLinkEnd) {
        const start = nodes.find(n => n.id === linkStartNodeId);
        if (start) {
          ctx.beginPath();
          ctx.moveTo(start.x, start.y);
          ctx.lineTo(tempLinkEnd.x, tempLinkEnd.y);
          ctx.strokeStyle = '#3b82f6';
          ctx.setLineDash([5, 5]);
          ctx.lineWidth = 2;
          ctx.stroke();
          ctx.setLineDash([]);
        }
      }

      // Nodes
      nodes.forEach(node => {
        // Draw pin/joint connector
        ctx.beginPath();
        if (node.isFixed) {
          // Triangle base for fixed ground
          ctx.moveTo(node.x, node.y);
          ctx.lineTo(node.x - 10, node.y + 15);
          ctx.lineTo(node.x + 10, node.y + 15);
          ctx.closePath();
          ctx.fillStyle = '#ef4444';
          ctx.fill();
          ctx.strokeStyle = '#b91c1c';
          ctx.lineWidth = 2;
          ctx.stroke();

          // Pin
          ctx.beginPath();
          ctx.arc(node.x, node.y, 6, 0, Math.PI * 2);
          ctx.fillStyle = '#fecaca';
          ctx.fill();
          ctx.stroke();

        } else {
          // Standard Joint Pin
          ctx.beginPath();
          ctx.arc(node.x, node.y, 7, 0, Math.PI * 2);
          // Color coding for motor vs normal
          if (node.isMotor) {
             ctx.fillStyle = node.isPathFollower ? '#ec4899' : '#fbbf24'; // Pink for Path Follower, Amber for Rotary
          } else {
             ctx.fillStyle = '#60a5fa';
          }
          
          ctx.fill();
          ctx.strokeStyle = '#1f2937';
          ctx.lineWidth = 1.5;
          ctx.stroke();
          
          // Inner detail
          ctx.beginPath();
          ctx.arc(node.x, node.y, 2, 0, Math.PI * 2);
          ctx.fillStyle = 'rgba(255,255,255,0.5)';
          ctx.fill();
        }

        // Tracer Visual Indicator
        if (node.isTracer) {
          ctx.beginPath();
          ctx.arc(node.x, node.y, 12, 0, Math.PI * 2);
          ctx.fillStyle = 'rgba(16, 185, 129, 0.2)'; // Green transparent halo
          ctx.fill();
          
          ctx.beginPath();
          ctx.arc(node.x, node.y, 22, 0, Math.PI * 2);
          ctx.strokeStyle = '#10b981';
          ctx.lineWidth = 1;
          ctx.setLineDash([2, 2]);
          ctx.stroke();
          ctx.setLineDash([]);

          // Crosshair
          ctx.beginPath();
          ctx.moveTo(node.x - 4, node.y);
          ctx.lineTo(node.x + 4, node.y);
          ctx.moveTo(node.x, node.y - 4);
          ctx.lineTo(node.x, node.y + 4);
          ctx.strokeStyle = '#059669';
          ctx.stroke();
        }

        // Hover Ring
        if (node.id === hoveredNodeId) {
          ctx.beginPath();
          ctx.arc(node.x, node.y, 14, 0, Math.PI * 2);
          ctx.strokeStyle = 'rgba(59, 130, 246, 0.5)';
          ctx.lineWidth = 3;
          ctx.stroke();
        }
        
        // Selection Ring
        if (node.id === selectedNodeId) {
          ctx.beginPath();
          ctx.arc(node.x, node.y, 16, 0, Math.PI * 2);
          ctx.strokeStyle = '#2563eb'; // Blue
          ctx.lineWidth = 2;
          ctx.setLineDash([3, 2]);
          ctx.stroke();
          ctx.setLineDash([]);
        }

        // Motor Visual (Arrow indicating rotation)
        if (node.isMotor && !node.isPathFollower) {
          ctx.beginPath();
          ctx.arc(node.x, node.y, 20, 0, Math.PI * 1.5);
          ctx.strokeStyle = 'rgba(245, 158, 11, 0.5)';
          ctx.lineWidth = 3;
          ctx.stroke();
        }

        // Path Follower Visual
        if (node.isMotor && node.isPathFollower) {
          ctx.beginPath();
          ctx.moveTo(node.x - 6, node.y - 6);
          ctx.lineTo(node.x + 6, node.y + 6);
          ctx.moveTo(node.x + 6, node.y - 6);
          ctx.lineTo(node.x - 6, node.y + 6);
          ctx.strokeStyle = '#ec4899'; // Pink
          ctx.lineWidth = 3;
          ctx.stroke();
        }
      });
      
      ctx.restore(); // Restore to screen coordinates (so future draws aren't affected if we added UI)

      // Update traces
      if (isPlaying && showTraces) {
        nodes.forEach(node => {
          if (!node.isFixed) {
            if (!tracesRef.current[node.id]) {
              tracesRef.current[node.id] = [];
            }
            const trace = tracesRef.current[node.id];
            const lastPoint = trace[trace.length - 1];
            if (!lastPoint || Math.hypot(lastPoint.x - node.x, lastPoint.y - node.y) > 2) {
              trace.push({ x: node.x, y: node.y });
              if (trace.length > 1000) {
                trace.shift();
              }
            }
          }
        });
      }

      if (isPlaying) {
        solveConstraints();
      }
      
      animationFrameRef.current = requestAnimationFrame(render);
    };

    render();
    return () => {
      if (animationFrameRef.current) cancelAnimationFrame(animationFrameRef.current);
    };
  }, [nodes, links, hoveredNodeId, draggedNodeId, linkStartNodeId, tempLinkEnd, isPlaying, solveConstraints, showTraces, targetPath, selectedNodeId, viewport]);


  // --- Handlers ---
  
  const handleAIInit = (mech: GeneratedMechanism) => {
    let generatedNodes = [...mech.nodes];

    // --- ALIGNMENT LOGIC ---
    // If a target path exists and the mechanism has a designated tracer node,
    // translate the entire mechanism so the tracer starts at the beginning of the path.
    if (targetPath.length > 0) {
      const tracerNode = generatedNodes.find(n => n.isTracer);
      if (tracerNode) {
        const startPoint = targetPath[0];
        const dx = startPoint.x - tracerNode.x;
        const dy = startPoint.y - tracerNode.y;
        
        generatedNodes = generatedNodes.map(n => ({
          ...n,
          x: n.x + dx,
          y: n.y + dy
        }));
      }
    }

    const newLinks: Link[] = mech.links.map(l => {
      const s = generatedNodes.find(n => n.id === l.sourceId);
      const t = generatedNodes.find(n => n.id === l.targetId);
      if (!s || !t) return null;
      return {
        id: nanoid(),
        sourceId: l.sourceId,
        targetId: l.targetId,
        length: Math.hypot(t.x - s.x, t.y - s.y)
      };
    }).filter(Boolean) as Link[];
    
    // Wire up motors to pivots after placement
    const newNodes = generatedNodes.map(n => {
      if (n.isMotor) {
        const link = newLinks.find(l => l.sourceId === n.id || l.targetId === n.id);
        if (link) {
           const otherId = link.sourceId === n.id ? link.targetId : link.sourceId;
           const other = generatedNodes.find(o => o.id === otherId);
           if (other && other.isFixed) {
             return {
               ...n,
               pivotId: other.id,
               radius: link.length,
               angle: Math.atan2(n.y - other.y, n.x - other.x)
             };
           }
        }
      }
      return n;
    });

    setNodes(newNodes);
    setLinks(newLinks);
    
    // Auto-start
    setIsPlaying(true);
    setShowTraces(true);
    tracesRef.current = {};
  };

  return (
    <div className="w-full h-screen flex flex-col overflow-hidden text-gray-800 font-sans">
      
      {/* Header / Info */}
      <div className="absolute top-4 left-4 z-10 bg-white/90 backdrop-blur p-4 rounded-xl shadow-sm border border-gray-200 max-w-sm pointer-events-none select-none">
        <h1 className="text-2xl font-bold bg-gradient-to-r from-blue-600 to-indigo-600 bg-clip-text text-transparent">
          LinkageLab
        </h1>
        <div className="mt-3 space-y-1">
          <div className="flex justify-between text-sm text-gray-600 gap-8">
             <span>Stats:</span>
             <span className="font-medium">{nodes.length} Joints • {links.length} Links</span>
          </div>
          <div className="flex justify-between text-sm items-center gap-8">
             <span>Mobility:</span>
             <span className={`font-bold ${mobilityInfo.color}`}>
                {mobilityInfo.status}
             </span>
          </div>
          {mobilityInfo.mobility > -99 && (
             <div className="text-xs text-right text-gray-400">
               M = {mobilityInfo.mobility}
             </div>
          )}
          <div className="text-xs text-gray-400 mt-2 border-t pt-1">
            Middle Click or Space + Drag to Pan • Wheel to Zoom
          </div>
        </div>
      </div>

      {/* Main Canvas */}
      <div 
        ref={containerRef} 
        className="flex-1 relative cursor-crosshair bg-[url('https://www.transparenttextures.com/patterns/graphy.png')]"
        onDragOver={handleDragOver}
        onDrop={handleDrop}
      >
        <canvas
          ref={canvasRef}
          className={`w-full h-full block ${isPanning ? 'cursor-grab active:cursor-grabbing' : ''}`}
          onMouseDown={handleCanvasMouseDown}
          onMouseMove={handleCanvasMouseMove}
          onMouseUp={handleCanvasMouseUp}
          onMouseLeave={handleCanvasMouseUp}
          onWheel={handleWheel}
        />
      </div>

      {/* Controls */}
      <Toolbar 
        activeTool={activeTool}
        onSelectTool={setActiveTool}
        isPlaying={isPlaying}
        onTogglePlay={() => setIsPlaying(!isPlaying)}
        onReset={() => {
           setIsPlaying(false);
           tracesRef.current = {};
           setNodes(nodes.map(n => ({...n}))); 
        }}
        onOpenAI={() => setIsAIModalOpen(true)}
        speed={speed}
        onSpeedChange={setSpeed}
        showTraces={showTraces}
        onToggleTraces={() => setShowTraces(!showTraces)}
        onToggleSidebar={() => setIsSidebarOpen(!isSidebarOpen)}
        isSidebarOpen={isSidebarOpen}
      />

      {/* Sidebar Library */}
      <Sidebar 
        isOpen={isSidebarOpen}
        onClose={() => setIsSidebarOpen(false)}
      />

      {/* Node Inspector Panel */}
      {selectedNode && (
        <NodeInspector 
          node={selectedNode}
          onUpdate={handleNodeUpdate}
          onClose={() => setSelectedNodeId(null)}
        />
      )}

      {/* Modals */}
      <AICreator 
        isOpen={isAIModalOpen} 
        onClose={() => setIsAIModalOpen(false)}
        onLoadMechanism={handleAIInit}
        targetPath={targetPath}
      />

    </div>
  );
};

export default App;
