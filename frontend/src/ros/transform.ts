import * as ROSLIB from 'roslib';
import { ros } from './init';

export let map = new ROSLIB.TFClient({
	ros : ros,
	fixedFrame : 'map',
	angularThres : 0.01,
	transThres : 0.01,
	rate : 10.0,
});
