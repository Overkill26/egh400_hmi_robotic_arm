def calculate_tax_australia(income):
    tax = 0

    # Define Australian tax brackets for 2023-2024
    brackets = [
        (0, 18200, 0),          # No tax for income between $0 and $18,200
        (18201, 45000, 0.19),   # 19% for income between $18,201 and $45,000
        (45001, 120000, 0.325), # 32.5% for income between $45,001 and $120,000
        (120001, 180000, 0.37), # 37% for income between $120,001 and $180,000
        (180001, float('inf'), 0.45)  # 45% for income above $180,001
    ]

    for lower, upper, rate in brackets:
        if income > lower:
            taxable_income = min(income, upper) - lower
            tax += taxable_income * rate
        else:
            break

    return tax

# Example usage
income = float(input("Enter your annual income: "))
tax = calculate_tax_australia(income)
print(f"Total tax to be paid: ${tax:.2f}")
