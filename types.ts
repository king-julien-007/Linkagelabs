
export enum ToolType {
  SELECT = 'SELECT',
  JOINT = 'JOINT',
  GROUND = 'GROUND',
  LINK = 'LINK',
  MOTOR = 'MOTOR',
  DRAW_PATH = 'DRAW_PATH',
  DELETE = 'DELETE'
}

export interface Node {
  id: string;
  x: number;
  y: number;
  isFixed: boolean;
  isMotor: boolean;
  isTracer?: boolean; // New property to identify the node tracing the path
  isPathFollower?: boolean; // New property: if true, motor follows target path instead of rotating around pivot
  // For Motor logic
  angle?: number; // Current angle if motor
  pivotId?: string; // ID of the fixed node this motor rotates around
  radius?: number; // Distance to pivot
  speed?: number; // Radians per frame
}

export interface Link {
  id: string;
  sourceId: string;
  targetId: string;
  length: number;
  color?: string;
}

export interface MechanismState {
  nodes: Node[];
  links: Link[];
}

// AI Generation Response Type
export interface GeneratedMechanism {
  name: string;
  description: string;
  nodes: { id: string; x: number; y: number; isFixed: boolean; isMotor: boolean; isTracer?: boolean }[];
  links: { sourceId: string; targetId: string }[];
}