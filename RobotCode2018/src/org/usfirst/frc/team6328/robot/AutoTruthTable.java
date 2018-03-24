package org.usfirst.frc.team6328.robot;

import org.usfirst.frc.team6328.robot.commands.SmartSideAuto.AutoDestination;

/**
 * Class to look up the proper auto for a given set of options
 * @author elliot
 *
 */
public class AutoTruthTable {
	
	// Bit assignments
	private static final byte switchSame = 0x01;
	private static final byte scaleSame = 0x02;
	private static final byte crossMiddleOK = 0x04;
	private static final byte switchPriority = 0x08;
	private static final byte scalePriority = 0x10;
	private static final byte twoCube = 0x20;
	private static final byte force = 0x40;
	
	/*
	 * Mask (first) has all values that matter for row, set (second) has all values that are 1
	 */
	private static final TableRow[] table = {
			new TableRow(switchSame|scaleSame|crossMiddleOK, 0, null, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|switchPriority|twoCube,
					crossMiddleOK, AutoDestination.SCALE_OPPOSITE, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|switchPriority|scalePriority,
					switchSame|crossMiddleOK|scalePriority, AutoDestination.SCALE_OPPOSITE, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|switchPriority|twoCube,
					crossMiddleOK|twoCube, AutoDestination.SCALE_OPPOSITE, AutoDestination.SWITCH_OPPOSITE),
			new TableRow(switchSame|crossMiddleOK|switchPriority|scalePriority,
					crossMiddleOK|switchPriority, AutoDestination.SWITCH_OPPOSITE, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|switchPriority,
					scaleSame, AutoDestination.SCALE_SAME, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|switchPriority|force,
					scaleSame|switchPriority|force, null, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|switchPriority|force,
					scaleSame|switchPriority, AutoDestination.SCALE_SAME, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|switchPriority|twoCube,
					scaleSame|crossMiddleOK, AutoDestination.SCALE_SAME, null),
			new TableRow(switchSame|scaleSame|switchPriority|twoCube,
					switchSame|scaleSame, AutoDestination.SCALE_SAME, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|switchPriority|twoCube,
					scaleSame|crossMiddleOK|twoCube, AutoDestination.SCALE_SAME, AutoDestination.SWITCH_OPPOSITE),
			new TableRow(switchSame|scaleSame|switchPriority|twoCube,
					switchSame|scaleSame|twoCube, AutoDestination.SCALE_SAME, AutoDestination.SWITCH_SAME),
			new TableRow(switchSame|scaleSame|crossMiddleOK|scalePriority,
					switchSame, AutoDestination.SWITCH_SAME, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|scalePriority|force,
					switchSame|scalePriority|force, null, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|scalePriority|force,
					switchSame|scalePriority, AutoDestination.SWITCH_SAME, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|scalePriority|twoCube,
					switchSame|crossMiddleOK, AutoDestination.SWITCH_SAME, null),
			new TableRow(switchSame|scaleSame|switchPriority|scalePriority,
					switchSame|scaleSame|switchPriority, AutoDestination.SWITCH_SAME, null),
			new TableRow(switchSame|scaleSame|crossMiddleOK|scalePriority|twoCube,
					switchSame|crossMiddleOK|twoCube, AutoDestination.SWITCH_SAME, AutoDestination.SCALE_OPPOSITE)
	};

	/**
	 * Find the auto that matches the parameters
	 * @param switchSame Whether the switch is on the same side as the robot
	 * @param scaleSame Whether the scale is on the same side as the robot
	 * @param crossMiddleOK Whether the robot can cross the middle of the field
	 * @param switchPriority Whether the switch has priority
	 * @param scalePriority Whether the scale has priority
	 * @param twoCube Whether to attempt a second cube
	 * @param force Whether to prevent ever not violating priority
	 * @return The two destinations. If just one destination is found, the second will be null. 
	 * If the first is null, it means cross line. If the return value itself is null, no match was found.
	 */
	public static SelectedAutoDestinations findRow(boolean switchSame, boolean scaleSame, boolean crossMiddleOK, 
			boolean switchPriority, boolean scalePriority, boolean twoCube, boolean force) {
		int state = 0;
		
		// Generate the state as a byte from the input
		if (switchSame) {
			state |= AutoTruthTable.switchSame;
		}
		if (scaleSame) {
			state |= AutoTruthTable.scaleSame;
		}
		if (crossMiddleOK) {
			state |= AutoTruthTable.crossMiddleOK;
		}
		if (switchPriority) {
			state |= AutoTruthTable.switchPriority;
		}
		if (scalePriority) {
			state |= AutoTruthTable.scalePriority;
		}
		if (twoCube) {
			state |= AutoTruthTable.twoCube;
		}
		if (force) {
			state |= AutoTruthTable.force;
		}
		
		TableRow row = null;
		for (int i= 0; i < table.length; i++) {
			TableRow currentRow = table[i];
			int maskedState = state & currentRow.getMask();
			if (maskedState == currentRow.getSet()) {
				row = currentRow;
				break;
			}
		}
		if (row != null) {
			return new SelectedAutoDestinations(row.getDest1(), row.getDest2());
		} else {
			return null;
		}
	}
	
	public static class SelectedAutoDestinations {
		
		private AutoDestination dest1, dest2;
		
		public SelectedAutoDestinations(AutoDestination dest1, AutoDestination dest2) {
			this.dest1 = dest1;
			this.dest2 = dest2;
		}

		public AutoDestination getDest1() {
			return dest1;
		}

		public AutoDestination getDest2() {
			return dest2;
		}
	}

	private static class TableRow {

		private int mask, set;
		private AutoDestination dest1, dest2;

		public TableRow(int mask, int set, AutoDestination dest1, AutoDestination dest2) {
			this.mask = mask;
			this.set = set;
			this.dest1 = dest1;
			this.dest2 = dest2;
		}

		public int getMask() {
			return mask;
		}

		public int getSet() {
			return set;
		}

		public AutoDestination getDest1() {
			return dest1;
		}

		public AutoDestination getDest2() {
			return dest2;
		}
	}
}