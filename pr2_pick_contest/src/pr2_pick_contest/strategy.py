import rospy


class PickingStrategy(object):
    def __init__(self, get_items, get_target_items, lookup_item):
        self._get_items = get_items
        self._get_target_items = get_target_items
        self._lookup_item = lookup_item

    def expected_value_of_bin(self, bin_id):
        bin_letter = bin_id[-1]
        items_response = self._get_items(bin_letter)
        items = items_response.items
        target_items_response = self._get_target_items(bin_letter)
        target_items = target_items_response.items
        if len(target_items) == 0 or len(items) == 0:
            return 0
        target_item = target_items[0]
        item_model_response = self._lookup_item(target_item)
        item_model = item_model_response.model

        bonus_points = item_model.bonus_points
        item_prior = item_model.success_prior

        points = 0
        num_items_prior = 1
        if len(items) == 0:
            points = 0
            num_items_prior = 0
        elif len(items) == 1:
            points = 10
            num_items_prior = 1
        elif len(items) == 2:
            points = 15
            num_items_prior = 0.95
        elif len(items) == 3:
            points = 20
            num_items_prior = 0.65
        else:
            points = 20
            num_items_prior = 0.2

        bin_prior = 1
        if bin_letter in 'ABC':
            bin_prior = 0.3
        else:
            bin_prior = 1

        prob = item_prior * bin_prior * num_items_prior
        value = (points + bonus_points) * prob
        rospy.loginfo(
            '{}: ({} + {}) * (p(item)={} * p(bin)={} * p(num_items)={}) = {}'.format(
                bin_id, points, bonus_points, item_prior, bin_prior,
                num_items_prior, value))

        return value

    def get_row_number(self, bin_letter):
        if bin_letter in 'ABC':
            row = 0
        elif bin_letter in 'DEF':
            row = 1
        elif bin_letter in 'GHI':
            row = 2
        else:
            row = 3
        return row

    def get_distance(self, start_letter, end_letter):
        row = 0
        start_row = self.get_row_number(start_letter)
        end_row = self.get_row_number(end_letter)
        return abs(start_row - end_row)

    def get_plan_by_expected_value(self):
        """Greedily go in order of expected value, where the expected value includes a
        fixed penalty for moving up and down.
        """
        bin_letters = 'ABCDEFGHIJKL'
        movement_penalty = 0.5 # Expected points lost per row that needs to be traversed.
        bin_values = []
        for bin_letter in bin_letters:
            bin_id = 'bin_{}'.format(bin_letter)
            bin_value = self.expected_value_of_bin(bin_id)
            bin_values.append((bin_letter, bin_value))
        bin_values = sorted(bin_values, key=lambda x: x[1], reverse=True)

        plan = []
        prev_letter = 'J'
        while len(bin_values) > 0:
            best_letter = None
            best_adjusted_value = None
            best_value = None
            for letter, value in bin_values:
                distance = self.get_distance(prev_letter, letter)
                adjusted_value = value - movement_penalty * distance
                if best_adjusted_value is None or adjusted_value > best_adjusted_value:
                    best_adjusted_value = adjusted_value
                    best_value = value
                    best_letter = letter
            plan.append((best_letter, best_value))
            prev_letter = best_letter
            bin_values.remove((best_letter, best_value))
        return plan

    def get_plan_row_by_row(self):
        """Go one row at a time.

        For each row, the robot visits the bins in order of expected value.
        It chooses which row to go to based on the value of the bins in that row.
        """
        rows = ['ABC', 'DEF', 'GHI', 'JKL']
        row_orders = [] # The order of the rows to visit.
        for row in rows:
            row_value = 0
            row_order = [] # The order of bins to visit within this row.
            for bin_letter in row:
                bin_id = 'bin_{}'.format(bin_letter)
                value_of_bin = self.expected_value_of_bin(bin_id)
                row_value += value_of_bin
                if value_of_bin > 0:
                    row_order.append((bin_id, value_of_bin))
            row_order = sorted(row_order, key=lambda x: x[1], reverse=True)
            row_orders.append((row_order, row_value))
        row_orders = sorted(row_orders, key=lambda x: x[1], reverse=True)

        plan = []
        for row_order, row_value in row_orders:
            rospy.loginfo('Order: {}, value: {}'.format(row_order, row_value))
            for bin_id, bin_value in row_order:
                plan.append((bin_id, bin_value))
        return plan
