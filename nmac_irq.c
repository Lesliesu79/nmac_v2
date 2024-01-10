#include "nmac.h"

static irqreturn_t nmac_irq_handler(int irqn, void *data)
{
	struct nmac_irq *irq = data;

	atomic_notifier_call_chain(&irq->nh, 0, NULL);

	return IRQ_HANDLED;
}

int nmac_irq_init_pcie(struct nmac_dev *mdev)
{
	struct pci_dev *pdev = mdev->pdev;
	struct device *dev = mdev->dev;
	int ret = 0;
	int k;

	// Allocate MSI IRQs
	mdev->irq_count = pci_alloc_irq_vectors(pdev, 1, NMAC_MAX_IRQ, PCI_IRQ_MSI | PCI_IRQ_MSIX);
	if (mdev->irq_count < 0) {
		dev_err(dev, "Failed to allocate IRQs");
		return -ENOMEM;
	}

	// Set up interrupts
	for (k = 0; k < mdev->irq_count; k++) {
		struct nmac_irq *irq;

		irq = kzalloc(sizeof(*irq), GFP_KERNEL);
		if (!irq) {
			ret = -ENOMEM;
			goto fail;
		}

		ATOMIC_INIT_NOTIFIER_HEAD(&irq->nh);

		ret = pci_request_irq(pdev, k, nmac_irq_handler, NULL,
				irq, "%s-%d", mdev->name, k);
		if (ret < 0) {
			kfree(irq);
			ret = -ENOMEM;
			dev_err(dev, "Failed to request IRQ %d", k);
			goto fail;
		}

		irq->index = k;
		irq->irqn = pci_irq_vector(pdev, k);
		mdev->irq[k] = irq;
	}

	dev_info(dev, "Configured %d IRQs", mdev->irq_count);

	return 0;
fail:
	nmac_irq_deinit_pcie(mdev);
	return ret;
}

void nmac_irq_deinit_pcie(struct nmac_dev *mdev)
{
	struct pci_dev *pdev = mdev->pdev;
	int k;

	for (k = 0; k < NMAC_MAX_IRQ; k++) {
		if (mdev->irq[k]) {
			pci_free_irq(pdev, k, mdev->irq[k]);
			kfree(mdev->irq[k]);
			mdev->irq[k] = NULL;
		}
	}

	pci_free_irq_vectors(pdev);
}

int nmac_irq_init_platform(struct nmac_dev *mdev)
{
	struct platform_device *pdev = mdev->pfdev;
	struct device *dev = mdev->dev;
	int k;

	// Allocate IRQs
	mdev->irq_count = platform_irq_count(pdev);

	// Set up interrupts
	for (k = 0; k < mdev->irq_count; k++) {
		int irqn;
		struct nmac_irq *irq;
		int ret;

		irqn = platform_get_irq(pdev, k);
		if (irqn < 0)
			return irqn;

		irq = devm_kzalloc(dev, sizeof(*irq), GFP_KERNEL);
		if (!irq)
			return -ENOMEM;

		ATOMIC_INIT_NOTIFIER_HEAD(&irq->nh);

		snprintf(irq->name, sizeof(irq->name), "%s-%u", mdev->name, k);
		ret = devm_request_irq(dev, irqn, nmac_irq_handler, 0, irq->name, irq);
		if (ret < 0) {
			dev_err(dev, "Failed to request IRQ %d (interrupt number %d)", k, irqn);
			return ret;
		}

		irq->index = k;
		irq->irqn = irqn;
		mdev->irq[k] = irq;
	}

	dev_info(dev, "Configured %d IRQs", mdev->irq_count);

	return 0;
}
